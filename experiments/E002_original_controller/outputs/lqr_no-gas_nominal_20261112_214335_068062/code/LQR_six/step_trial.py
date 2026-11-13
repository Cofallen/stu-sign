"""Deterministic W+J trial on the isolated 10 cm platform."""
import json

import numpy as np


class StepTrial:
    thresholds = dict(takeoff_gap_m=[.05, .45], takeoff_vz_min=.05,
                      airborne_min_s=.10, top_normal_z_min=.8,
                      top_contact_z_min=.095, top_wheel_edge_margin_m=.06,
                      stable_s=2., stable_tilt_rad=.05, stable_speed_m_s=.05,
                      max_closure_m=1e-4)

    def __init__(self, model, jump_x):
        self.jump_x = jump_x
        self.platform = model.geom('step10_single').id
        center = float(model.geom_pos[self.platform, 0])
        half_length = float(model.geom_size[self.platform, 0])
        self.edge = center + half_length
        self.far_edge = center - half_length
        self.floor = model.geom('floor').id
        self.wheels = [model.geom(f'{side}_wheel_Link_collision_0').id for side in ('l', 'r')]
        self.triggered = self.braking = False
        self.samples = []

    def sample(self, data, tilt, teleop):
        ground, top, side, body = set(), set(), False, False
        platform_contacts = []
        for contact in data.contact:
            a, b = map(int, contact.geom)
            if b in (self.floor, self.platform):
                a, b = b, a
            if a not in (self.floor, self.platform):
                continue
            if a == self.platform:
                platform_contacts.append(dict(geom=b, position_m=contact.pos.tolist(),
                                              normal=contact.frame[:3].tolist(),
                                              distance_m=float(contact.dist)))
            if b not in self.wheels:
                body = True
            elif a == self.floor:
                ground.add(b)
            elif (abs(contact.frame[2]) >= self.thresholds['top_normal_z_min']
                  and contact.pos[2] >= self.thresholds['top_contact_z_min']):
                top.add(b)
            else:
                side = True
        wheel_x = data.geom_xpos[self.wheels, 0]
        self.samples.append(dict(t=data.time, x=float(data.qpos[0]), wheel_x=wheel_x.tolist(),
                                 vz=float(data.qvel[2]), speed=float(np.linalg.norm(data.qvel[:3])),
                                 tilt=tilt, ground=len(ground), top=len(top), side=side, body=body,
                                 platform_contacts=platform_contacts))
        # Release W at the first top contact so the existing brake can settle on the platform.
        self.braking = self.braking or bool(top)
        teleop.set_pressed(set() if self.braking or data.time < 1. else {'w'})
        if not self.triggered and data.qpos[0] <= self.jump_x:
            teleop.on_key('j')
            self.triggered = True
            self.request_index = len(self.samples) - 1

    def finish(self, out, result):
        p = self.thresholds
        samples = self.samples
        after_request = samples[self.request_index:] if self.triggered else []
        takeoff = next((s for s in after_request if not (s['ground'] or s['top'] or s['side'] or s['body'])
                        and s['vz'] > p['takeoff_vz_min']), None)
        after_takeoff = samples[samples.index(takeoff)+1:] if takeoff else []
        first_contact = next((s for s in after_takeoff if s['ground'] or s['top'] or s['side'] or s['body']), None)
        air_s = first_contact['t'] - takeoff['t'] if first_contact else None
        gap = min(takeoff['wheel_x']) - self.edge if takeoff else None
        stable = [s for s in samples if s['t'] >= samples[-1]['t'] - p['stable_s']]
        checks = dict(takeoff=gap is not None and p['takeoff_gap_m'][0] <= gap <= p['takeoff_gap_m'][1]
                      and air_s is not None and air_s >= p['airborne_min_s'],
                      top_landing=bool(first_contact and first_contact['top'] and not first_contact['ground']
                                       and not first_contact['side'] and not first_contact['body']
                                       and any(s['top'] == 2 for s in after_takeoff)),
                      clean_contact=not any(s['side'] or s['body'] for s in samples),
                      stable=bool(first_contact and first_contact['t'] < stable[0]['t']
                                  and all(s['top'] == 2 and not s['ground']
                                          and min(s['wheel_x']) >= self.far_edge+p['top_wheel_edge_margin_m']
                                          and max(s['wheel_x']) <= self.edge-p['top_wheel_edge_margin_m']
                                          and s['tilt'] < p['stable_tilt_rad']
                                          and s['speed'] < p['stable_speed_m_s'] for s in stable)),
                      closure=result['max_closure_m'] < p['max_closure_m'])
        with (out/'step_samples.jsonl').open('w') as stream:
            for sample in samples:
                stream.write(json.dumps(sample)+'\n')
        return dict(thresholds=p, checks=checks, request=after_request[0] if after_request else None, takeoff=takeoff, takeoff_gap_m=gap, airborne_s=air_s,
                    first_contact=first_contact, final=samples[-1],
                    stable_peak_speed_m_s=max(s['speed'] for s in stable),
                    stable_peak_tilt_rad=max(s['tilt'] for s in stable))

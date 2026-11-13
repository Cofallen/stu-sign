#define WP_NO_BFLOAT16

#define WP_TILE_BLOCK_DIM 256
#define WP_NO_CRT
#include "builtin.h"
#include "deterministic.h"

// Map wp.breakpoint() to a device brkpt at the call site so cuda-gdb attributes the stop to the generated .cu line
#if defined(__CUDACC__) && !defined(_MSC_VER)
#define __debugbreak() __brkpt()
#endif

#define builtin_tid1d() wp::tid(_idx, dim)
#define builtin_tid2d(x, y) wp::tid(x, y, _idx, dim)
#define builtin_tid3d(x, y, z) wp::tid(x, y, z, _idx, dim)
#define builtin_tid4d(x, y, z, w) wp::tid(x, y, z, w, _idx, dim)

#define builtin_block_dim() wp::block_dim()

// CUDA Thread Block Cluster shape declaration. Expands to __cluster_dims__
// only on devices that support clusters (compute capability 9.0+); otherwise
// expands to nothing so the same source compiles cleanly for any target arch.
#if defined(__CUDA_ARCH__) && (__CUDA_ARCH__ >= 900)
#define WP_CLUSTER_DIMS(x, y, z) __cluster_dims__(x, y, z)
#else
#define WP_CLUSTER_DIMS(x, y, z)
#endif

// Maximum registers per thread. __maxnreg__ was added in CUDA Toolkit 12.4;
// older toolkits ignore the opt-in so the same source remains compilable.
#if defined(__CUDACC_VER_MAJOR__) && (__CUDACC_VER_MAJOR__ > 12 || (__CUDACC_VER_MAJOR__ == 12 && __CUDACC_VER_MINOR__ >= 4))
#define WP_MAXNREG(n) __maxnreg__(n)
#else
#define WP_MAXNREG(n)
#endif

// Allow PTXAS to spill registers into shared memory. CUDA Toolkit 13.0
// introduced the pragma, which is unavailable in device-debug compilation.
#if defined(__CUDACC_VER_MAJOR__) && (__CUDACC_VER_MAJOR__ >= 13) && !defined(_DEBUG)
#define WP_ENABLE_SMEM_SPILLING() asm volatile(".pragma \"enable_smem_spilling\";");
#else
#define WP_ENABLE_SMEM_SPILLING()
#endif


// avoid namespacing of float type for casting to float type, this is to avoid wp::float(x), which is not valid in C++
#define float(x) cast_float(x)
#define adj_float(x, adj_x, adj_ret) adj_cast_float(x, adj_x, adj_ret)

#define int(x) cast_int(x)
#define adj_int(x, adj_x, adj_ret) adj_cast_int(x, adj_x, adj_ret)


// /home/cofallen/miniconda3/envs/mjlab/lib/python3.12/site-packages/mujoco_warp/_src/support.py:497
static CUDA_CALLABLE void jac_dof_0(
    wp::array_t<wp::int32> var_body_parentid,
    wp::array_t<wp::int32> var_body_rootid,
    wp::array_t<wp::int32> var_dof_bodyid,
    wp::array_t<wp::int32> var_body_isdofancestor,
    wp::array_t<wp::vec_t<3, wp::float32>> var_subtree_com_in,
    wp::array_t<wp::vec_t<6, wp::float32>> var_cdof_in,
    wp::vec_t<3, wp::float32> var_point,
    wp::int32 var_bodyid,
    wp::int32 var_dofid,
    wp::int32 var_worldid,
    wp::vec_t<3, wp::float32> & ret_0,
    wp::vec_t<3, wp::float32> & ret_1)
{
    //---------
    // primal vars
    wp::int32* var_0;
    const wp::int32 var_1 = 0;
    bool var_2;
    wp::int32 var_3;
    const wp::float32 var_4 = 0.0;
    wp::vec_t<3, wp::float32> var_5;
    const wp::float32 var_6 = 0.0;
    wp::vec_t<3, wp::float32> var_7;
    wp::int32* var_8;
    wp::int32 var_9;
    wp::vec_t<3, wp::float32>* var_10;
    wp::vec_t<3, wp::float32> var_11;
    wp::vec_t<3, wp::float32> var_12;
    wp::vec_t<3, wp::float32> var_13;
    wp::vec_t<6, wp::float32>* var_14;
    wp::vec_t<6, wp::float32> var_15;
    wp::vec_t<6, wp::float32> var_16;
    wp::vec_t<3, wp::float32> var_17;
    wp::vec_t<3, wp::float32> var_18;
    wp::vec_t<3, wp::float32> var_19;
    wp::vec_t<3, wp::float32> var_20;
    wp::vec_t<3, wp::float32> var_21;
    //---------
    // forward
    // def jac_dof(                                                                           <L 498>
    // if body_isdofancestor[bodyid, dofid] == 0:                                             <L 513>
    var_0 = wp::address(var_body_isdofancestor, var_bodyid, var_dofid);
    var_3 = wp::load(var_0);
    var_2 = (var_3 == var_1);
    if (var_2) {
        // return wp.vec3(0.0), wp.vec3(0.0)                                                  <L 514>
        var_5 = wp::vec_t<3, wp::float32>(var_4);
        var_7 = wp::vec_t<3, wp::float32>(var_6);
        ret_0 = var_5;
        ret_1 = var_7;
        return;
    }
    // offset = point - wp.vec3(subtree_com_in[worldid, body_rootid[bodyid]])                 <L 516>
    var_8 = wp::address(var_body_rootid, var_bodyid);
    var_9 = wp::load(var_8);
    var_10 = wp::address(var_subtree_com_in, var_worldid, var_9);
    var_12 = wp::load(var_10);
    var_11 = wp::vec_t<3, wp::float32>(var_12);
    var_13 = wp::sub(var_point, var_11);
    // cdof = cdof_in[worldid, dofid]                                                         <L 518>
    var_14 = wp::address(var_cdof_in, var_worldid, var_dofid);
    var_16 = wp::load(var_14);
    var_15 = wp::copy(var_16);
    // cdof_ang = wp.spatial_top(cdof)                                                        <L 519>
    var_17 = wp::spatial_top(var_15);
    // cdof_lin = wp.spatial_bottom(cdof)                                                     <L 520>
    var_18 = wp::spatial_bottom(var_15);
    // jacp = cdof_lin + wp.cross(cdof_ang, offset)                                           <L 522>
    var_19 = wp::cross(var_17, var_13);
    var_20 = wp::add(var_18, var_19);
    // jacr = cdof_ang                                                                        <L 523>
    var_21 = wp::copy(var_17);
    // return jacp, jacr                                                                      <L 525>
    ret_0 = var_20;
    ret_1 = var_21;
    return;
}


// /home/cofallen/miniconda3/envs/mjlab/lib/python3.12/site-packages/mujoco_warp/_src/math.py:133
static CUDA_CALLABLE wp::vec_t<6, wp::float32> motion_cross_0(
    wp::vec_t<6, wp::float32> var_u,
    wp::vec_t<6, wp::float32> var_v)
{
    //---------
    // primal vars
    const wp::int32 var_0 = 0;
    wp::float32 var_1;
    const wp::int32 var_2 = 1;
    wp::float32 var_3;
    const wp::int32 var_4 = 2;
    wp::float32 var_5;
    wp::vec_t<3, wp::float32> var_6;
    const wp::int32 var_7 = 3;
    wp::float32 var_8;
    const wp::int32 var_9 = 4;
    wp::float32 var_10;
    const wp::int32 var_11 = 5;
    wp::float32 var_12;
    wp::vec_t<3, wp::float32> var_13;
    const wp::int32 var_14 = 0;
    wp::float32 var_15;
    const wp::int32 var_16 = 1;
    wp::float32 var_17;
    const wp::int32 var_18 = 2;
    wp::float32 var_19;
    wp::vec_t<3, wp::float32> var_20;
    const wp::int32 var_21 = 3;
    wp::float32 var_22;
    const wp::int32 var_23 = 4;
    wp::float32 var_24;
    const wp::int32 var_25 = 5;
    wp::float32 var_26;
    wp::vec_t<3, wp::float32> var_27;
    wp::vec_t<3, wp::float32> var_28;
    wp::vec_t<3, wp::float32> var_29;
    wp::vec_t<3, wp::float32> var_30;
    wp::vec_t<3, wp::float32> var_31;
    wp::vec_t<6, wp::float32> var_32;
    //---------
    // forward
    // def motion_cross(u: wp.spatial_vector, v: wp.spatial_vector) -> wp.spatial_vector:       <L 134>
    // u0 = wp.vec3(u[0], u[1], u[2])                                                         <L 136>
    var_1 = wp::extract(var_u, var_0);
    var_3 = wp::extract(var_u, var_2);
    var_5 = wp::extract(var_u, var_4);
    var_6 = wp::vec_t<3, wp::float32>(var_1, var_3, var_5);
    // u1 = wp.vec3(u[3], u[4], u[5])                                                         <L 137>
    var_8 = wp::extract(var_u, var_7);
    var_10 = wp::extract(var_u, var_9);
    var_12 = wp::extract(var_u, var_11);
    var_13 = wp::vec_t<3, wp::float32>(var_8, var_10, var_12);
    // v0 = wp.vec3(v[0], v[1], v[2])                                                         <L 138>
    var_15 = wp::extract(var_v, var_14);
    var_17 = wp::extract(var_v, var_16);
    var_19 = wp::extract(var_v, var_18);
    var_20 = wp::vec_t<3, wp::float32>(var_15, var_17, var_19);
    // v1 = wp.vec3(v[3], v[4], v[5])                                                         <L 139>
    var_22 = wp::extract(var_v, var_21);
    var_24 = wp::extract(var_v, var_23);
    var_26 = wp::extract(var_v, var_25);
    var_27 = wp::vec_t<3, wp::float32>(var_22, var_24, var_26);
    // ang = wp.cross(u0, v0)                                                                 <L 141>
    var_28 = wp::cross(var_6, var_20);
    // vel = wp.cross(u1, v0) + wp.cross(u0, v1)                                              <L 142>
    var_29 = wp::cross(var_13, var_20);
    var_30 = wp::cross(var_6, var_27);
    var_31 = wp::add(var_29, var_30);
    // return wp.spatial_vector(ang, vel)                                                     <L 144>
    var_32 = wp::vec_t<6, wp::float32>(var_28, var_31);
    return var_32;
}


// /home/cofallen/miniconda3/envs/mjlab/lib/python3.12/site-packages/mujoco_warp/_src/support.py:607
static CUDA_CALLABLE void jac_dot_dof_0(
    wp::array_t<wp::int32> var_body_parentid,
    wp::array_t<wp::int32> var_body_rootid,
    wp::array_t<wp::int32> var_jnt_type,
    wp::array_t<wp::int32> var_jnt_dofadr,
    wp::array_t<wp::int32> var_dof_bodyid,
    wp::array_t<wp::int32> var_dof_jntid,
    wp::array_t<wp::int32> var_body_isdofancestor,
    wp::array_t<wp::vec_t<3, wp::float32>> var_subtree_com_in,
    wp::array_t<wp::vec_t<6, wp::float32>> var_cdof_in,
    wp::array_t<wp::vec_t<6, wp::float32>> var_cvel_in,
    wp::array_t<wp::vec_t<6, wp::float32>> var_cdof_dot_in,
    wp::vec_t<3, wp::float32> var_point,
    wp::int32 var_bodyid,
    wp::int32 var_dofid,
    wp::int32 var_worldid,
    wp::vec_t<3, wp::float32> & ret_0,
    wp::vec_t<3, wp::float32> & ret_1)
{
    //---------
    // primal vars
    wp::int32* var_0;
    const wp::int32 var_1 = 0;
    bool var_2;
    wp::int32 var_3;
    const wp::float32 var_4 = 0.0;
    wp::vec_t<3, wp::float32> var_5;
    const wp::float32 var_6 = 0.0;
    wp::vec_t<3, wp::float32> var_7;
    wp::int32* var_8;
    wp::int32 var_9;
    wp::vec_t<3, wp::float32>* var_10;
    wp::vec_t<3, wp::float32> var_11;
    wp::vec_t<3, wp::float32> var_12;
    wp::vec_t<3, wp::float32> var_13;
    wp::vec_t<6, wp::float32>* var_14;
    wp::vec_t<6, wp::float32> var_15;
    wp::vec_t<6, wp::float32> var_16;
    wp::vec_t<3, wp::float32> var_17;
    wp::vec_t<3, wp::float32> var_18;
    wp::vec_t<3, wp::float32> var_19;
    wp::vec_t<3, wp::float32> var_20;
    wp::vec_t<6, wp::float32>* var_21;
    wp::vec_t<6, wp::float32> var_22;
    wp::vec_t<6, wp::float32> var_23;
    wp::vec_t<6, wp::float32>* var_24;
    wp::vec_t<6, wp::float32> var_25;
    wp::vec_t<6, wp::float32> var_26;
    wp::int32* var_27;
    wp::int32 var_28;
    wp::int32 var_29;
    wp::int32* var_30;
    wp::int32 var_31;
    wp::int32 var_32;
    wp::int32* var_33;
    wp::int32 var_34;
    wp::int32 var_35;
    bool var_36;
    const wp::int32 var_37 = 1;
    bool var_38;
    bool var_39;
    const wp::int32 var_40 = 0;
    bool var_41;
    const wp::int32 var_42 = 3;
    wp::int32 var_43;
    bool var_44;
    wp::int32* var_45;
    wp::int32 var_46;
    wp::vec_t<6, wp::float32>* var_47;
    wp::vec_t<6, wp::float32> var_48;
    wp::vec_t<6, wp::float32> var_49;
    wp::vec_t<6, wp::float32> var_50;
    wp::vec_t<6, wp::float32> var_51;
    wp::vec_t<6, wp::float32> var_52;
    wp::vec_t<3, wp::float32> var_53;
    wp::vec_t<3, wp::float32> var_54;
    wp::vec_t<3, wp::float32> var_55;
    wp::vec_t<3, wp::float32> var_56;
    wp::vec_t<3, wp::float32> var_57;
    wp::vec_t<3, wp::float32> var_58;
    wp::vec_t<3, wp::float32> var_59;
    wp::vec_t<3, wp::float32> var_60;
    //---------
    // forward
    // def jac_dot_dof(                                                                       <L 608>
    // if body_isdofancestor[bodyid, dofid] == 0:                                             <L 628>
    var_0 = wp::address(var_body_isdofancestor, var_bodyid, var_dofid);
    var_3 = wp::load(var_0);
    var_2 = (var_3 == var_1);
    if (var_2) {
        // return wp.vec3(0.0), wp.vec3(0.0)                                                  <L 629>
        var_5 = wp::vec_t<3, wp::float32>(var_4);
        var_7 = wp::vec_t<3, wp::float32>(var_6);
        ret_0 = var_5;
        ret_1 = var_7;
        return;
    }
    // com = subtree_com_in[worldid, body_rootid[bodyid]]                                     <L 631>
    var_8 = wp::address(var_body_rootid, var_bodyid);
    var_9 = wp::load(var_8);
    var_10 = wp::address(var_subtree_com_in, var_worldid, var_9);
    var_12 = wp::load(var_10);
    var_11 = wp::copy(var_12);
    // offset = point - com                                                                   <L 632>
    var_13 = wp::sub(var_point, var_11);
    // cvel = cvel_in[worldid, bodyid]                                                        <L 635>
    var_14 = wp::address(var_cvel_in, var_worldid, var_bodyid);
    var_16 = wp::load(var_14);
    var_15 = wp::copy(var_16);
    // pvel_lin = wp.spatial_bottom(cvel) - wp.cross(offset, wp.spatial_top(cvel))            <L 636>
    var_17 = wp::spatial_bottom(var_15);
    var_18 = wp::spatial_top(var_15);
    var_19 = wp::cross(var_13, var_18);
    var_20 = wp::sub(var_17, var_19);
    // cdof = cdof_in[worldid, dofid]                                                         <L 638>
    var_21 = wp::address(var_cdof_in, var_worldid, var_dofid);
    var_23 = wp::load(var_21);
    var_22 = wp::copy(var_23);
    // cdof_dot = cdof_dot_in[worldid, dofid]                                                 <L 639>
    var_24 = wp::address(var_cdof_dot_in, var_worldid, var_dofid);
    var_26 = wp::load(var_24);
    var_25 = wp::copy(var_26);
    // dofjntid = dof_jntid[dofid]                                                            <L 642>
    var_27 = wp::address(var_dof_jntid, var_dofid);
    var_29 = wp::load(var_27);
    var_28 = wp::copy(var_29);
    // jnttype = jnt_type[dofjntid]                                                           <L 643>
    var_30 = wp::address(var_jnt_type, var_28);
    var_32 = wp::load(var_30);
    var_31 = wp::copy(var_32);
    // jntdofadr = jnt_dofadr[dofjntid]                                                       <L 644>
    var_33 = wp::address(var_jnt_dofadr, var_28);
    var_35 = wp::load(var_33);
    var_34 = wp::copy(var_35);
    // if (jnttype == JointType.BALL) or ((jnttype == JointType.FREE) and dofid >= jntdofadr + 3):       <L 646>
    var_38 = (var_31 == var_37);
    var_36 = var_38;
    if (!var_36) {
        var_41 = (var_31 == var_40);
        var_39 = var_41;
        if (var_39) {
            var_43 = wp::add(var_34, var_42);
            var_44 = (var_dofid >= var_43);
            var_39 = var_39 && var_44;
        }
        var_36 = var_36 || var_39;
    }
    if (var_36) {
        // cvel = cvel_in[worldid, dof_bodyid[dofid]]                                         <L 648>
        var_45 = wp::address(var_dof_bodyid, var_dofid);
        var_46 = wp::load(var_45);
        var_47 = wp::address(var_cvel_in, var_worldid, var_46);
        var_49 = wp::load(var_47);
        var_48 = wp::copy(var_49);
        // cdof_dot = motion_cross(cvel, cdof)                                                <L 649>
        var_50 = motion_cross_0(var_48, var_22);
    }
    var_51 = wp::where(var_36, var_48, var_15);
    var_52 = wp::where(var_36, var_50, var_25);
    // cdof_dot_ang = wp.spatial_top(cdof_dot)                                                <L 651>
    var_53 = wp::spatial_top(var_52);
    // cdof_dot_lin = wp.spatial_bottom(cdof_dot)                                             <L 652>
    var_54 = wp::spatial_bottom(var_52);
    // correction1 = wp.cross(cdof_dot_ang, offset)                                           <L 656>
    var_55 = wp::cross(var_53, var_13);
    // correction2 = wp.cross(wp.spatial_top(cdof), pvel_lin)                                 <L 659>
    var_56 = wp::spatial_top(var_22);
    var_57 = wp::cross(var_56, var_20);
    // jacp = cdof_dot_lin + correction1 + correction2                                        <L 661>
    var_58 = wp::add(var_54, var_55);
    var_59 = wp::add(var_58, var_57);
    // jacr = cdof_dot_ang                                                                    <L 662>
    var_60 = wp::copy(var_53);
    // return jacp, jacr                                                                      <L 664>
    ret_0 = var_59;
    ret_1 = var_60;
    return;
}


// /home/cofallen/miniconda3/envs/mjlab/lib/python3.12/site-packages/mujoco_warp/_src/constraint.py:82
static CUDA_CALLABLE void _efc_row_0(
    wp::int32 var_opt_disableflags,
    wp::int32 var_worldid,
    wp::float32 var_timestep,
    wp::int32 var_efcid,
    wp::float32 var_pos_aref,
    wp::float32 var_pos_imp,
    wp::float32 var_invweight,
    wp::vec_t<2, wp::float32> var_solref,
    wp::vec_t<5, wp::float32> var_solimp,
    wp::float32 var_margin,
    wp::float32 var_vel,
    wp::float32 var_frictionloss,
    wp::int32 var_type,
    wp::int32 var_id,
    wp::array_t<wp::int32> var_type_out,
    wp::array_t<wp::int32> var_id_out,
    wp::array_t<wp::float32> var_pos_out,
    wp::array_t<wp::float32> var_margin_out,
    wp::array_t<wp::float32> var_D_out,
    wp::array_t<wp::float32> var_vel_out,
    wp::array_t<wp::float32> var_aref_out,
    wp::array_t<wp::float32> var_frictionloss_out)
{
    //---------
    // primal vars
    const wp::int32 var_0 = 0;
    wp::float32 var_1;
    const wp::int32 var_2 = 1;
    wp::float32 var_3;
    const wp::int32 var_4 = 0;
    wp::float32 var_5;
    const wp::int32 var_6 = 1;
    wp::float32 var_7;
    const wp::int32 var_8 = 2;
    wp::float32 var_9;
    const wp::int32 var_10 = 3;
    wp::float32 var_11;
    const wp::int32 var_12 = 4;
    wp::float32 var_13;
    const wp::int32 var_14 = 4096;
    wp::int32 var_15;
    bool var_16;
    const wp::float32 var_17 = 2.0;
    wp::float32 var_18;
    wp::float32 var_19;
    wp::float32 var_20;
    const wp::float32 var_21 = 0.0001;
    const wp::float32 var_22 = 0.0001;
    const wp::float32 var_23 = 0.9999;
    const wp::float32 var_24 = 0.9999;
    wp::float32 var_25;
    const wp::float32 var_26 = 0.0001;
    const wp::float32 var_27 = 0.0001;
    const wp::float32 var_28 = 0.9999;
    const wp::float32 var_29 = 0.9999;
    wp::float32 var_30;
    const wp::float32 var_31 = 1e-15;
    const wp::float32 var_32 = 1e-15;
    wp::float32 var_33;
    const wp::float32 var_34 = 0.0001;
    const wp::float32 var_35 = 0.0001;
    const wp::float32 var_36 = 0.9999;
    const wp::float32 var_37 = 0.9999;
    wp::float32 var_38;
    const wp::float32 var_39 = 1.0;
    wp::float32 var_40;
    wp::float32 var_41;
    const wp::float32 var_42 = 1.0;
    wp::float32 var_43;
    wp::float32 var_44;
    wp::float32 var_45;
    wp::float32 var_46;
    wp::float32 var_47;
    const wp::float32 var_48 = 2.0;
    wp::float32 var_49;
    wp::float32 var_50;
    const wp::int32 var_51 = 0;
    wp::float32 var_52;
    const wp::int32 var_53 = 0;
    bool var_54;
    const wp::int32 var_55 = 0;
    wp::float32 var_56;
    wp::float32 var_57;
    wp::float32 var_58;
    wp::float32 var_59;
    const wp::int32 var_60 = 1;
    wp::float32 var_61;
    const wp::int32 var_62 = 0;
    bool var_63;
    const wp::int32 var_64 = 1;
    wp::float32 var_65;
    wp::float32 var_66;
    wp::float32 var_67;
    wp::float32 var_68;
    wp::float32 var_69;
    wp::float32 var_70;
    const wp::float32 var_71 = 1.0;
    const wp::float32 var_72 = 1.0;
    wp::float32 var_73;
    wp::float32 var_74;
    wp::float32 var_75;
    wp::float32 var_76;
    wp::float32 var_77;
    const wp::float32 var_78 = 1.0;
    const wp::float32 var_79 = 1.0;
    const wp::float32 var_80 = 1.0;
    wp::float32 var_81;
    const wp::float32 var_82 = 1.0;
    wp::float32 var_83;
    wp::float32 var_84;
    wp::float32 var_85;
    const wp::float32 var_86 = 1.0;
    wp::float32 var_87;
    wp::float32 var_88;
    wp::float32 var_89;
    wp::float32 var_90;
    bool var_91;
    wp::float32 var_92;
    wp::float32 var_93;
    wp::float32 var_94;
    wp::float32 var_95;
    wp::float32 var_96;
    const wp::float32 var_97 = 1.0;
    bool var_98;
    wp::float32 var_99;
    const wp::float32 var_100 = 1.0;
    const wp::float32 var_101 = 1.0;
    wp::float32 var_102;
    wp::float32 var_103;
    wp::float32 var_104;
    const wp::float32 var_105 = 1e-15;
    const wp::float32 var_106 = 1e-15;
    wp::float32 var_107;
    wp::float32 var_108;
    wp::float32 var_109;
    wp::float32 var_110;
    wp::float32 var_111;
    wp::float32 var_112;
    wp::float32 var_113;
    wp::float32 var_114;
    //---------
    // forward
    // def _efc_row(                                                                          <L 83>
    // timeconst = solref[0]                                                                  <L 111>
    var_1 = wp::extract(var_solref, var_0);
    // dampratio = solref[1]                                                                  <L 112>
    var_3 = wp::extract(var_solref, var_2);
    // dmin = solimp[0]                                                                       <L 113>
    var_5 = wp::extract(var_solimp, var_4);
    // dmax = solimp[1]                                                                       <L 114>
    var_7 = wp::extract(var_solimp, var_6);
    // width = solimp[2]                                                                      <L 115>
    var_9 = wp::extract(var_solimp, var_8);
    // mid = solimp[3]                                                                        <L 116>
    var_11 = wp::extract(var_solimp, var_10);
    // power = solimp[4]                                                                      <L 117>
    var_13 = wp::extract(var_solimp, var_12);
    // if not (opt_disableflags & DisableBit.REFSAFE):                                        <L 119>
    var_15 = wp::bit_and(var_opt_disableflags, var_14);
    var_16 = wp::unot(var_15);
    if (var_16) {
        // timeconst = wp.max(timeconst, 2.0 * timestep)                                      <L 120>
        var_18 = wp::mul(var_17, var_timestep);
        var_19 = wp::max(var_1, var_18);
    }
    var_20 = wp::where(var_16, var_19, var_1);
    // dmin = wp.clamp(dmin, types.MJ_MINIMP, types.MJ_MAXIMP)                                <L 122>
    var_25 = wp::clamp(var_5, var_22, var_24);
    // dmax = wp.clamp(dmax, types.MJ_MINIMP, types.MJ_MAXIMP)                                <L 123>
    var_30 = wp::clamp(var_7, var_27, var_29);
    // width = wp.max(types.MJ_MINVAL, width)                                                 <L 124>
    var_33 = wp::max(var_32, var_9);
    // mid = wp.clamp(mid, types.MJ_MINIMP, types.MJ_MAXIMP)                                  <L 125>
    var_38 = wp::clamp(var_11, var_35, var_37);
    // power = wp.max(1.0, power)                                                             <L 126>
    var_40 = wp::max(var_39, var_13);
    // dmax_sq = dmax * dmax                                                                  <L 129>
    var_41 = wp::mul(var_30, var_30);
    // k = 1.0 / (dmax_sq * timeconst * timeconst * dampratio * dampratio)                    <L 130>
    var_43 = wp::mul(var_41, var_20);
    var_44 = wp::mul(var_43, var_20);
    var_45 = wp::mul(var_44, var_3);
    var_46 = wp::mul(var_45, var_3);
    var_47 = wp::div(var_42, var_46);
    // b = 2.0 / (dmax * timeconst)                                                           <L 131>
    var_49 = wp::mul(var_30, var_20);
    var_50 = wp::div(var_48, var_49);
    // k = wp.where(solref[0] <= 0, -solref[0] / dmax_sq, k)                                  <L 132>
    var_52 = wp::extract(var_solref, var_51);
    var_54 = (var_52 <= var_53);
    var_56 = wp::extract(var_solref, var_55);
    var_57 = wp::neg(var_56);
    var_58 = wp::div(var_57, var_41);
    var_59 = wp::where(var_54, var_58, var_47);
    // b = wp.where(solref[1] <= 0, -solref[1] / dmax, b)                                     <L 133>
    var_61 = wp::extract(var_solref, var_60);
    var_63 = (var_61 <= var_62);
    var_65 = wp::extract(var_solref, var_64);
    var_66 = wp::neg(var_65);
    var_67 = wp::div(var_66, var_30);
    var_68 = wp::where(var_63, var_67, var_50);
    // imp_x = wp.abs(pos_imp) / width                                                        <L 135>
    var_69 = wp::abs(var_pos_imp);
    var_70 = wp::div(var_69, var_33);
    // imp_a = (1.0 / wp.pow(mid, power - 1.0)) * wp.pow(imp_x, power)                        <L 136>
    var_73 = wp::sub(var_40, var_72);
    var_74 = wp::pow(var_38, var_73);
    var_75 = wp::div(var_71, var_74);
    var_76 = wp::pow(var_70, var_40);
    var_77 = wp::mul(var_75, var_76);
    // imp_b = 1.0 - (1.0 / wp.pow(1.0 - mid, power - 1.0)) * wp.pow(1.0 - imp_x, power)       <L 137>
    var_81 = wp::sub(var_80, var_38);
    var_83 = wp::sub(var_40, var_82);
    var_84 = wp::pow(var_81, var_83);
    var_85 = wp::div(var_79, var_84);
    var_87 = wp::sub(var_86, var_70);
    var_88 = wp::pow(var_87, var_40);
    var_89 = wp::mul(var_85, var_88);
    var_90 = wp::sub(var_78, var_89);
    // imp_y = wp.where(imp_x < mid, imp_a, imp_b)                                            <L 138>
    var_91 = (var_70 < var_38);
    var_92 = wp::where(var_91, var_77, var_90);
    // imp = dmin + imp_y * (dmax - dmin)                                                     <L 139>
    var_93 = wp::sub(var_30, var_25);
    var_94 = wp::mul(var_92, var_93);
    var_95 = wp::add(var_25, var_94);
    // imp = wp.clamp(imp, dmin, dmax)                                                        <L 140>
    var_96 = wp::clamp(var_95, var_25, var_30);
    // imp = wp.where(imp_x > 1.0, dmax, imp)                                                 <L 141>
    var_98 = (var_70 > var_97);
    var_99 = wp::where(var_98, var_30, var_96);
    // D_out[worldid, efcid] = 1.0 / wp.max(invweight * (1.0 - imp) / imp, types.MJ_MINVAL)       <L 144>
    var_102 = wp::sub(var_101, var_99);
    var_103 = wp::mul(var_invweight, var_102);
    var_104 = wp::div(var_103, var_99);
    var_107 = wp::max(var_104, var_106);
    var_108 = wp::div(var_100, var_107);
    wp::array_store(var_D_out, var_worldid, var_efcid, var_108);
    // vel_out[worldid, efcid] = vel                                                          <L 145>
    wp::array_store(var_vel_out, var_worldid, var_efcid, var_vel);
    // aref_out[worldid, efcid] = -k * imp * pos_aref - b * vel                               <L 146>
    var_109 = wp::neg(var_59);
    var_110 = wp::mul(var_109, var_99);
    var_111 = wp::mul(var_110, var_pos_aref);
    var_112 = wp::mul(var_68, var_vel);
    var_113 = wp::sub(var_111, var_112);
    wp::array_store(var_aref_out, var_worldid, var_efcid, var_113);
    // pos_out[worldid, efcid] = pos_aref + margin                                            <L 147>
    var_114 = wp::add(var_pos_aref, var_margin);
    wp::array_store(var_pos_out, var_worldid, var_efcid, var_114);
    // margin_out[worldid, efcid] = margin                                                    <L 148>
    wp::array_store(var_margin_out, var_worldid, var_efcid, var_margin);
    // frictionloss_out[worldid, efcid] = frictionloss                                        <L 149>
    wp::array_store(var_frictionloss_out, var_worldid, var_efcid, var_frictionloss);
    // type_out[worldid, efcid] = type                                                        <L 150>
    wp::array_store(var_type_out, var_worldid, var_efcid, var_type);
    // id_out[worldid, efcid] = id                                                            <L 151>
    wp::array_store(var_id_out, var_worldid, var_efcid, var_id);
}


// /home/cofallen/miniconda3/envs/mjlab/lib/python3.12/site-packages/mujoco_warp/_src/support.py:497
static CUDA_CALLABLE void adj_jac_dof_0(
    wp::array_t<wp::int32> var_body_parentid,
    wp::array_t<wp::int32> var_body_rootid,
    wp::array_t<wp::int32> var_dof_bodyid,
    wp::array_t<wp::int32> var_body_isdofancestor,
    wp::array_t<wp::vec_t<3, wp::float32>> var_subtree_com_in,
    wp::array_t<wp::vec_t<6, wp::float32>> var_cdof_in,
    wp::vec_t<3, wp::float32> var_point,
    wp::int32 var_bodyid,
    wp::int32 var_dofid,
    wp::int32 var_worldid,
    wp::vec_t<3, wp::float32> & ret_0,
    wp::vec_t<3, wp::float32> & ret_1,
    wp::array_t<wp::int32> & adj_body_parentid,
    wp::array_t<wp::int32> & adj_body_rootid,
    wp::array_t<wp::int32> & adj_dof_bodyid,
    wp::array_t<wp::int32> & adj_body_isdofancestor,
    wp::array_t<wp::vec_t<3, wp::float32>> & adj_subtree_com_in,
    wp::array_t<wp::vec_t<6, wp::float32>> & adj_cdof_in,
    wp::vec_t<3, wp::float32> & adj_point,
    wp::int32 & adj_bodyid,
    wp::int32 & adj_dofid,
    wp::int32 & adj_worldid,
    wp::vec_t<3, wp::float32> & adj_ret_0,
    wp::vec_t<3, wp::float32> & adj_ret_1)
{
	// reverse mode disabled (no backward-enabled kernel depends on this function)
}


// /home/cofallen/miniconda3/envs/mjlab/lib/python3.12/site-packages/mujoco_warp/_src/math.py:133
static CUDA_CALLABLE void adj_motion_cross_0(
    wp::vec_t<6, wp::float32> var_u,
    wp::vec_t<6, wp::float32> var_v,
    wp::vec_t<6, wp::float32> & adj_u,
    wp::vec_t<6, wp::float32> & adj_v,
    wp::vec_t<6, wp::float32> & adj_ret)
{
	// reverse mode disabled (no backward-enabled kernel depends on this function)
}


// /home/cofallen/miniconda3/envs/mjlab/lib/python3.12/site-packages/mujoco_warp/_src/support.py:607
static CUDA_CALLABLE void adj_jac_dot_dof_0(
    wp::array_t<wp::int32> var_body_parentid,
    wp::array_t<wp::int32> var_body_rootid,
    wp::array_t<wp::int32> var_jnt_type,
    wp::array_t<wp::int32> var_jnt_dofadr,
    wp::array_t<wp::int32> var_dof_bodyid,
    wp::array_t<wp::int32> var_dof_jntid,
    wp::array_t<wp::int32> var_body_isdofancestor,
    wp::array_t<wp::vec_t<3, wp::float32>> var_subtree_com_in,
    wp::array_t<wp::vec_t<6, wp::float32>> var_cdof_in,
    wp::array_t<wp::vec_t<6, wp::float32>> var_cvel_in,
    wp::array_t<wp::vec_t<6, wp::float32>> var_cdof_dot_in,
    wp::vec_t<3, wp::float32> var_point,
    wp::int32 var_bodyid,
    wp::int32 var_dofid,
    wp::int32 var_worldid,
    wp::vec_t<3, wp::float32> & ret_0,
    wp::vec_t<3, wp::float32> & ret_1,
    wp::array_t<wp::int32> & adj_body_parentid,
    wp::array_t<wp::int32> & adj_body_rootid,
    wp::array_t<wp::int32> & adj_jnt_type,
    wp::array_t<wp::int32> & adj_jnt_dofadr,
    wp::array_t<wp::int32> & adj_dof_bodyid,
    wp::array_t<wp::int32> & adj_dof_jntid,
    wp::array_t<wp::int32> & adj_body_isdofancestor,
    wp::array_t<wp::vec_t<3, wp::float32>> & adj_subtree_com_in,
    wp::array_t<wp::vec_t<6, wp::float32>> & adj_cdof_in,
    wp::array_t<wp::vec_t<6, wp::float32>> & adj_cvel_in,
    wp::array_t<wp::vec_t<6, wp::float32>> & adj_cdof_dot_in,
    wp::vec_t<3, wp::float32> & adj_point,
    wp::int32 & adj_bodyid,
    wp::int32 & adj_dofid,
    wp::int32 & adj_worldid,
    wp::vec_t<3, wp::float32> & adj_ret_0,
    wp::vec_t<3, wp::float32> & adj_ret_1)
{
	// reverse mode disabled (no backward-enabled kernel depends on this function)
}


// /home/cofallen/miniconda3/envs/mjlab/lib/python3.12/site-packages/mujoco_warp/_src/constraint.py:82
static CUDA_CALLABLE void adj__efc_row_0(
    wp::int32 var_opt_disableflags,
    wp::int32 var_worldid,
    wp::float32 var_timestep,
    wp::int32 var_efcid,
    wp::float32 var_pos_aref,
    wp::float32 var_pos_imp,
    wp::float32 var_invweight,
    wp::vec_t<2, wp::float32> var_solref,
    wp::vec_t<5, wp::float32> var_solimp,
    wp::float32 var_margin,
    wp::float32 var_vel,
    wp::float32 var_frictionloss,
    wp::int32 var_type,
    wp::int32 var_id,
    wp::array_t<wp::int32> var_type_out,
    wp::array_t<wp::int32> var_id_out,
    wp::array_t<wp::float32> var_pos_out,
    wp::array_t<wp::float32> var_margin_out,
    wp::array_t<wp::float32> var_D_out,
    wp::array_t<wp::float32> var_vel_out,
    wp::array_t<wp::float32> var_aref_out,
    wp::array_t<wp::float32> var_frictionloss_out,
    wp::int32 & adj_opt_disableflags,
    wp::int32 & adj_worldid,
    wp::float32 & adj_timestep,
    wp::int32 & adj_efcid,
    wp::float32 & adj_pos_aref,
    wp::float32 & adj_pos_imp,
    wp::float32 & adj_invweight,
    wp::vec_t<2, wp::float32> & adj_solref,
    wp::vec_t<5, wp::float32> & adj_solimp,
    wp::float32 & adj_margin,
    wp::float32 & adj_vel,
    wp::float32 & adj_frictionloss,
    wp::int32 & adj_type,
    wp::int32 & adj_id,
    wp::array_t<wp::int32> & adj_type_out,
    wp::array_t<wp::int32> & adj_id_out,
    wp::array_t<wp::float32> & adj_pos_out,
    wp::array_t<wp::float32> & adj_margin_out,
    wp::array_t<wp::float32> & adj_D_out,
    wp::array_t<wp::float32> & adj_vel_out,
    wp::array_t<wp::float32> & adj_aref_out,
    wp::array_t<wp::float32> & adj_frictionloss_out)
{
	// reverse mode disabled (no backward-enabled kernel depends on this function)
}



extern "C" __global__ void _equality_connect__locals__kernel_27a1bbc0_cuda_kernel_forward(
    wp::launch_bounds_t<2> dim,
    wp::int32 var_nv,
    wp::int32 var_nsite,
    wp::array_t<wp::float32> var_opt_timestep,
    wp::int32 var_opt_disableflags,
    wp::array_t<wp::int32> var_body_parentid,
    wp::array_t<wp::int32> var_body_rootid,
    wp::array_t<wp::int32> var_body_weldid,
    wp::array_t<wp::int32> var_body_dofnum,
    wp::array_t<wp::int32> var_body_dofadr,
    wp::array_t<wp::vec_t<2, wp::float32>> var_body_invweight0,
    wp::array_t<wp::int32> var_jnt_type,
    wp::array_t<wp::int32> var_jnt_dofadr,
    wp::array_t<wp::int32> var_dof_bodyid,
    wp::array_t<wp::int32> var_dof_jntid,
    wp::array_t<wp::int32> var_dof_parentid,
    wp::array_t<wp::int32> var_site_bodyid,
    wp::array_t<wp::int32> var_eq_obj1id,
    wp::array_t<wp::int32> var_eq_obj2id,
    wp::array_t<wp::int32> var_eq_objtype,
    wp::array_t<wp::vec_t<2, wp::float32>> var_eq_solref,
    wp::array_t<wp::vec_t<5, wp::float32>> var_eq_solimp,
    wp::array_t<wp::vec_t<11, wp::float32>> var_eq_data,
    wp::array_t<wp::int32> var_body_isdofancestor,
    wp::array_t<wp::int32> var_eq_connect_adr,
    wp::array_t<wp::float32> var_qvel_in,
    wp::array_t<bool> var_eq_active_in,
    wp::array_t<wp::vec_t<3, wp::float32>> var_xpos_in,
    wp::array_t<wp::mat_t<3, 3, wp::float32>> var_xmat_in,
    wp::array_t<wp::vec_t<3, wp::float32>> var_site_xpos_in,
    wp::array_t<wp::vec_t<3, wp::float32>> var_subtree_com_in,
    wp::array_t<wp::vec_t<6, wp::float32>> var_cdof_in,
    wp::array_t<wp::vec_t<6, wp::float32>> var_cvel_in,
    wp::array_t<wp::vec_t<6, wp::float32>> var_cdof_dot_in,
    wp::array_t<wp::vec_t<3, wp::float32>> var_subtree_linvel_in,
    wp::int32 var_njmax_in,
    wp::int32 var_njmax_nnz_in,
    wp::array_t<wp::int32> var_ne_out,
    wp::array_t<wp::int32> var_nefc_out,
    wp::array_t<wp::int32> var_efc_type_out,
    wp::array_t<wp::int32> var_efc_id_out,
    wp::array_t<wp::int32> var_efc_jtdaj_adr_out,
    wp::array_t<wp::int32> var_efc_jtdaj_nrow_out,
    wp::array_t<wp::int32> var_efc_jtdaj_nblock_out,
    wp::array_t<wp::int32> var_efc_J_rownnz_out,
    wp::array_t<wp::int32> var_efc_J_rowadr_out,
    wp::array_t<wp::int32> var_efc_J_colind_out,
    wp::array_t<wp::float32> var_efc_J_out,
    wp::array_t<wp::float32> var_efc_pos_out,
    wp::array_t<wp::float32> var_efc_margin_out,
    wp::array_t<wp::float32> var_efc_D_out,
    wp::array_t<wp::float32> var_efc_vel_out,
    wp::array_t<wp::float32> var_efc_aref_out,
    wp::array_t<wp::float32> var_efc_frictionloss_out,
    wp::array_t<wp::int32> var_efc_nnz_out)
{
    wp::tile_shared_storage_t tile_mem;

    for (size_t _idx = static_cast<size_t>(blockDim.x) * static_cast<size_t>(blockIdx.x) + static_cast<size_t>(threadIdx.x);
         _idx < dim.size;
         _idx += static_cast<size_t>(blockDim.x) * static_cast<size_t>(gridDim.x))
    {
            // reset shared memory allocator
        wp::tile_shared_storage_t::init();

        //---------
        // primal vars
        wp::int32 var_0;
        wp::int32 var_1;
        wp::int32* var_2;
        wp::int32 var_3;
        wp::int32 var_4;
        bool* var_5;
        bool var_6;
        bool var_7;
        const wp::int32 var_8 = 3;
        wp::int32 var_9;
        const wp::int32 var_10 = 3;
        wp::int32 var_11;
        const wp::int32 var_12 = 3;
        wp::int32 var_13;
        bool var_14;
        const bool var_15 = false;
        const wp::int32 var_16 = 0;
        wp::int32 var_17;
        const wp::int32 var_18 = 1;
        wp::int32 var_19;
        const wp::int32 var_20 = 2;
        wp::int32 var_21;
        wp::shape_t* var_22;
        const wp::int32 var_23 = 0;
        wp::int32 var_24;
        wp::shape_t var_25;
        wp::int32 var_26;
        wp::vec_t<11, wp::float32>* var_27;
        wp::vec_t<11, wp::float32> var_28;
        wp::vec_t<11, wp::float32> var_29;
        const wp::int32 var_30 = 0;
        wp::float32 var_31;
        const wp::int32 var_32 = 1;
        wp::float32 var_33;
        const wp::int32 var_34 = 2;
        wp::float32 var_35;
        wp::vec_t<3, wp::float32> var_36;
        const wp::int32 var_37 = 3;
        wp::float32 var_38;
        const wp::int32 var_39 = 4;
        wp::float32 var_40;
        const wp::int32 var_41 = 5;
        wp::float32 var_42;
        wp::vec_t<3, wp::float32> var_43;
        wp::int32* var_44;
        wp::int32 var_45;
        wp::int32 var_46;
        wp::int32* var_47;
        wp::int32 var_48;
        wp::int32 var_49;
        bool var_50;
        const wp::int32 var_51 = 0;
        bool var_52;
        wp::int32* var_53;
        const wp::int32 var_54 = 6;
        bool var_55;
        wp::int32 var_56;
        wp::int32* var_57;
        wp::int32 var_58;
        wp::int32 var_59;
        wp::int32* var_60;
        wp::int32 var_61;
        wp::int32 var_62;
        wp::vec_t<3, wp::float32>* var_63;
        wp::vec_t<3, wp::float32> var_64;
        wp::vec_t<3, wp::float32> var_65;
        wp::vec_t<3, wp::float32>* var_66;
        wp::vec_t<3, wp::float32> var_67;
        wp::vec_t<3, wp::float32> var_68;
        wp::int32 var_69;
        wp::int32 var_70;
        wp::vec_t<3, wp::float32>* var_71;
        wp::mat_t<3, 3, wp::float32>* var_72;
        wp::vec_t<3, wp::float32> var_73;
        wp::mat_t<3, 3, wp::float32> var_74;
        wp::vec_t<3, wp::float32> var_75;
        wp::vec_t<3, wp::float32> var_76;
        wp::vec_t<3, wp::float32>* var_77;
        wp::mat_t<3, 3, wp::float32>* var_78;
        wp::vec_t<3, wp::float32> var_79;
        wp::mat_t<3, 3, wp::float32> var_80;
        wp::vec_t<3, wp::float32> var_81;
        wp::vec_t<3, wp::float32> var_82;
        wp::int32 var_83;
        wp::int32 var_84;
        wp::vec_t<3, wp::float32> var_85;
        wp::vec_t<3, wp::float32> var_86;
        wp::vec_t<3, wp::float32> var_87;
        const wp::float32 var_88 = 0.0;
        const wp::float32 var_89 = 0.0;
        const wp::float32 var_90 = 0.0;
        wp::vec_t<3, wp::float32> var_91;
        const wp::float32 var_92 = 0.0;
        const wp::float32 var_93 = 0.0;
        const wp::float32 var_94 = 0.0;
        wp::vec_t<3, wp::float32> var_95;
        const bool var_96 = false;
        wp::range_t var_97;
        wp::int32 var_98;
        wp::vec_t<3, wp::float32> var_99;
        wp::vec_t<3, wp::float32> var_100;
        wp::vec_t<3, wp::float32> var_101;
        wp::vec_t<3, wp::float32> var_102;
        wp::vec_t<3, wp::float32> var_103;
        wp::vec_t<3, wp::float32> var_104;
        wp::vec_t<3, wp::float32> var_105;
        wp::vec_t<3, wp::float32> var_106;
        wp::vec_t<3, wp::float32> var_107;
        wp::vec_t<3, wp::float32> var_108;
        const wp::int32 var_109 = 0;
        wp::float32 var_110;
        const wp::int32 var_111 = 1;
        wp::float32 var_112;
        const wp::int32 var_113 = 2;
        wp::float32 var_114;
        wp::float32* var_115;
        wp::float32 var_116;
        wp::float32 var_117;
        wp::vec_t<3, wp::float32> var_118;
        wp::vec_t<3, wp::float32> var_119;
        wp::vec_t<3, wp::float32> var_120;
        wp::vec_t<3, wp::float32> var_121;
        wp::shape_t* var_122;
        const wp::int32 var_123 = 0;
        wp::int32 var_124;
        wp::shape_t var_125;
        wp::int32 var_126;
        wp::vec_t<2, wp::float32>* var_127;
        const wp::int32 var_128 = 0;
        wp::float32 var_129;
        wp::vec_t<2, wp::float32> var_130;
        wp::vec_t<2, wp::float32>* var_131;
        const wp::int32 var_132 = 0;
        wp::float32 var_133;
        wp::vec_t<2, wp::float32> var_134;
        wp::float32 var_135;
        wp::float32 var_136;
        wp::shape_t* var_137;
        const wp::int32 var_138 = 0;
        wp::int32 var_139;
        wp::shape_t var_140;
        wp::int32 var_141;
        wp::vec_t<2, wp::float32>* var_142;
        wp::vec_t<2, wp::float32> var_143;
        wp::vec_t<2, wp::float32> var_144;
        wp::shape_t* var_145;
        const wp::int32 var_146 = 0;
        wp::int32 var_147;
        wp::shape_t var_148;
        wp::int32 var_149;
        wp::vec_t<5, wp::float32>* var_150;
        wp::vec_t<5, wp::float32> var_151;
        wp::vec_t<5, wp::float32> var_152;
        wp::shape_t* var_153;
        const wp::int32 var_154 = 0;
        wp::int32 var_155;
        wp::shape_t var_156;
        wp::int32 var_157;
        wp::float32* var_158;
        wp::float32 var_159;
        wp::float32 var_160;
        const wp::int32 var_161 = 0;
        wp::int32 var_162;
        wp::float32 var_163;
        const wp::float32 var_164 = 0.0;
        wp::float32 var_165;
        const wp::float32 var_166 = 0.0;
        const wp::int32 var_167 = 0;
        const wp::int32 var_168 = 0;
        wp::float32 var_169;
        wp::float32 var_170;
        const wp::int32 var_171 = 1;
        wp::int32 var_172;
        wp::float32 var_173;
        const wp::float32 var_174 = 0.0;
        wp::float32 var_175;
        const wp::float32 var_176 = 0.0;
        const wp::int32 var_177 = 0;
        const wp::int32 var_178 = 0;
        wp::float32 var_179;
        wp::float32 var_180;
        const wp::int32 var_181 = 2;
        wp::int32 var_182;
        wp::float32 var_183;
        const wp::float32 var_184 = 0.0;
        wp::float32 var_185;
        const wp::float32 var_186 = 0.0;
        const wp::int32 var_187 = 0;
        const wp::int32 var_188 = 0;
        wp::float32 var_189;
        wp::float32 var_190;
        //---------
        // forward
        // def kernel(                                                                            <L 157>
        // worldid, eqconnectid = wp.tid()                                                        <L 218>
        builtin_tid2d(var_0, var_1);
        // eqid = eq_connect_adr[eqconnectid]                                                     <L 219>
        var_2 = wp::address(var_eq_connect_adr, var_1);
        var_4 = wp::load(var_2);
        var_3 = wp::copy(var_4);
        // if not eq_active_in[worldid, eqid]:                                                    <L 221>
        var_5 = wp::address(var_eq_active_in, var_0, var_3);
        var_7 = wp::load(var_5);
        var_6 = wp::unot(var_7);
        if (var_6) {
            // return                                                                             <L 222>
            continue;
        }
        // wp.atomic_add(ne_out, worldid, 3)                                                      <L 224>
        var_9 = wp::atomic_add(var_ne_out, var_0, var_8);
        // efcid = wp.atomic_add(nefc_out, worldid, 3)                                            <L 225>
        var_11 = wp::atomic_add(var_nefc_out, var_0, var_10);
        // if efcid >= njmax_in - 3:                                                              <L 227>
        var_13 = wp::sub(var_njmax_in, var_12);
        var_14 = (var_11 >= var_13);
        if (var_14) {
            // return                                                                             <L 228>
            continue;
        }
        // if wp.static(is_sparse and newton):                                                    <L 230>
        // efcid0 = efcid + 0                                                                     <L 235>
        var_17 = wp::add(var_11, var_16);
        // efcid1 = efcid + 1                                                                     <L 236>
        var_19 = wp::add(var_11, var_18);
        // efcid2 = efcid + 2                                                                     <L 237>
        var_21 = wp::add(var_11, var_20);
        // data = eq_data[worldid % eq_data.shape[0], eqid]                                       <L 239>
        var_22 = &(var_eq_data.shape);
        var_25 = wp::load(var_22);
        var_24 = wp::extract(var_25, var_23);
        var_26 = wp::mod(var_0, var_24);
        var_27 = wp::address(var_eq_data, var_26, var_3);
        var_29 = wp::load(var_27);
        var_28 = wp::copy(var_29);
        // anchor1 = wp.vec3f(data[0], data[1], data[2])                                          <L 240>
        var_31 = wp::extract(var_28, var_30);
        var_33 = wp::extract(var_28, var_32);
        var_35 = wp::extract(var_28, var_34);
        var_36 = wp::vec_t<3, wp::float32>(var_31, var_33, var_35);
        // anchor2 = wp.vec3f(data[3], data[4], data[5])                                          <L 241>
        var_38 = wp::extract(var_28, var_37);
        var_40 = wp::extract(var_28, var_39);
        var_42 = wp::extract(var_28, var_41);
        var_43 = wp::vec_t<3, wp::float32>(var_38, var_40, var_42);
        // obj1id = eq_obj1id[eqid]                                                               <L 243>
        var_44 = wp::address(var_eq_obj1id, var_3);
        var_46 = wp::load(var_44);
        var_45 = wp::copy(var_46);
        // obj2id = eq_obj2id[eqid]                                                               <L 244>
        var_47 = wp::address(var_eq_obj2id, var_3);
        var_49 = wp::load(var_47);
        var_48 = wp::copy(var_49);
        // if nsite > 0 and eq_objtype[eqid] == types.ObjType.SITE:                               <L 246>
        var_52 = (var_nsite > var_51);
        var_50 = var_52;
        if (var_50) {
            var_53 = wp::address(var_eq_objtype, var_3);
            var_56 = wp::load(var_53);
            var_55 = (var_56 == var_54);
            var_50 = var_50 && var_55;
        }
        if (var_50) {
            // body1 = site_bodyid[obj1id]                                                        <L 247>
            var_57 = wp::address(var_site_bodyid, var_45);
            var_59 = wp::load(var_57);
            var_58 = wp::copy(var_59);
            // body2 = site_bodyid[obj2id]                                                        <L 248>
            var_60 = wp::address(var_site_bodyid, var_48);
            var_62 = wp::load(var_60);
            var_61 = wp::copy(var_62);
            // pos1 = site_xpos_in[worldid, obj1id]                                               <L 249>
            var_63 = wp::address(var_site_xpos_in, var_0, var_45);
            var_65 = wp::load(var_63);
            var_64 = wp::copy(var_65);
            // pos2 = site_xpos_in[worldid, obj2id]                                               <L 250>
            var_66 = wp::address(var_site_xpos_in, var_0, var_48);
            var_68 = wp::load(var_66);
            var_67 = wp::copy(var_68);
        }
        if (!var_50) {
            // body1 = obj1id                                                                     <L 252>
            var_69 = wp::copy(var_45);
            // body2 = obj2id                                                                     <L 253>
            var_70 = wp::copy(var_48);
            // pos1 = xpos_in[worldid, body1] + xmat_in[worldid, body1] @ anchor1                 <L 254>
            var_71 = wp::address(var_xpos_in, var_0, var_69);
            var_72 = wp::address(var_xmat_in, var_0, var_69);
            var_74 = wp::load(var_72);
            var_73 = wp::mul(var_74, var_36);
            var_76 = wp::load(var_71);
            var_75 = wp::add(var_76, var_73);
            // pos2 = xpos_in[worldid, body2] + xmat_in[worldid, body2] @ anchor2                 <L 255>
            var_77 = wp::address(var_xpos_in, var_0, var_70);
            var_78 = wp::address(var_xmat_in, var_0, var_70);
            var_80 = wp::load(var_78);
            var_79 = wp::mul(var_80, var_43);
            var_82 = wp::load(var_77);
            var_81 = wp::add(var_82, var_79);
        }
        var_83 = wp::where(var_50, var_58, var_69);
        var_84 = wp::where(var_50, var_61, var_70);
        var_85 = wp::where(var_50, var_64, var_75);
        var_86 = wp::where(var_50, var_67, var_81);
        // pos = pos1 - pos2                                                                      <L 258>
        var_87 = wp::sub(var_85, var_86);
        // Jqvel = wp.vec3f(0.0, 0.0, 0.0)                                                        <L 261>
        var_91 = wp::vec_t<3, wp::float32>(var_88, var_89, var_90);
        // Jdotv = wp.vec3f(0.0, 0.0, 0.0)                                                        <L 262>
        var_95 = wp::vec_t<3, wp::float32>(var_92, var_93, var_94);
        // if wp.static(is_sparse):                                                               <L 264>
        // for dofid in range(nv):                                                                <L 386>
        var_97 = wp::range(var_nv);
        start_for_2:;
            if (iter_cmp(var_97) == 0) goto end_for_2;
            var_98 = wp::iter_next(var_97);
            // jacp1, _ = support.jac_dof(                                                        <L 387>
            // body_parentid,                                                                     <L 388>
            // body_rootid,                                                                       <L 389>
            // dof_bodyid,                                                                        <L 390>
            // body_isdofancestor,                                                                <L 391>
            // subtree_com_in,                                                                    <L 392>
            // cdof_in,                                                                           <L 393>
            // pos1,                                                                              <L 394>
            // body1,                                                                             <L 395>
            // dofid,                                                                             <L 396>
            // worldid,                                                                           <L 397>
            jac_dof_0(var_body_parentid, var_body_rootid, var_dof_bodyid, var_body_isdofancestor, var_subtree_com_in, var_cdof_in, var_85, var_83, var_98, var_0, var_99, var_100);
            // jacp2, _ = support.jac_dof(                                                        <L 399>
            // body_parentid,                                                                     <L 400>
            // body_rootid,                                                                       <L 401>
            // dof_bodyid,                                                                        <L 402>
            // body_isdofancestor,                                                                <L 403>
            // subtree_com_in,                                                                    <L 404>
            // cdof_in,                                                                           <L 405>
            // pos2,                                                                              <L 406>
            // body2,                                                                             <L 407>
            // dofid,                                                                             <L 408>
            // worldid,                                                                           <L 409>
            jac_dof_0(var_body_parentid, var_body_rootid, var_dof_bodyid, var_body_isdofancestor, var_subtree_com_in, var_cdof_in, var_86, var_84, var_98, var_0, var_101, var_102);
            // j1mj2 = jacp1 - jacp2                                                              <L 411>
            var_103 = wp::sub(var_99, var_101);
            // jacp1_dot, _ = support.jac_dot_dof(                                                <L 413>
            // body_parentid,                                                                     <L 414>
            // body_rootid,                                                                       <L 415>
            // jnt_type,                                                                          <L 416>
            // jnt_dofadr,                                                                        <L 417>
            // dof_bodyid,                                                                        <L 418>
            // dof_jntid,                                                                         <L 419>
            // body_isdofancestor,                                                                <L 420>
            // subtree_com_in,                                                                    <L 421>
            // cdof_in,                                                                           <L 422>
            // cvel_in,                                                                           <L 423>
            // cdof_dot_in,                                                                       <L 424>
            // pos1,                                                                              <L 425>
            // body1,                                                                             <L 426>
            // dofid,                                                                             <L 427>
            // worldid,                                                                           <L 428>
            jac_dot_dof_0(var_body_parentid, var_body_rootid, var_jnt_type, var_jnt_dofadr, var_dof_bodyid, var_dof_jntid, var_body_isdofancestor, var_subtree_com_in, var_cdof_in, var_cvel_in, var_cdof_dot_in, var_85, var_83, var_98, var_0, var_104, var_105);
            // jacp2_dot, _ = support.jac_dot_dof(                                                <L 430>
            // body_parentid,                                                                     <L 431>
            // body_rootid,                                                                       <L 432>
            // jnt_type,                                                                          <L 433>
            // jnt_dofadr,                                                                        <L 434>
            // dof_bodyid,                                                                        <L 435>
            // dof_jntid,                                                                         <L 436>
            // body_isdofancestor,                                                                <L 437>
            // subtree_com_in,                                                                    <L 438>
            // cdof_in,                                                                           <L 439>
            // cvel_in,                                                                           <L 440>
            // cdof_dot_in,                                                                       <L 441>
            // pos2,                                                                              <L 442>
            // body2,                                                                             <L 443>
            // dofid,                                                                             <L 444>
            // worldid,                                                                           <L 445>
            jac_dot_dof_0(var_body_parentid, var_body_rootid, var_jnt_type, var_jnt_dofadr, var_dof_bodyid, var_dof_jntid, var_body_isdofancestor, var_subtree_com_in, var_cdof_in, var_cvel_in, var_cdof_dot_in, var_86, var_84, var_98, var_0, var_106, var_107);
            // j1mj2_dot = jacp1_dot - jacp2_dot                                                  <L 447>
            var_108 = wp::sub(var_104, var_106);
            // efc_J_out[worldid, efcid0, dofid] = j1mj2[0]                                       <L 449>
            var_110 = wp::extract(var_103, var_109);
            wp::array_store(var_efc_J_out, var_0, var_17, var_98, var_110);
            // efc_J_out[worldid, efcid1, dofid] = j1mj2[1]                                       <L 450>
            var_112 = wp::extract(var_103, var_111);
            wp::array_store(var_efc_J_out, var_0, var_19, var_98, var_112);
            // efc_J_out[worldid, efcid2, dofid] = j1mj2[2]                                       <L 451>
            var_114 = wp::extract(var_103, var_113);
            wp::array_store(var_efc_J_out, var_0, var_21, var_98, var_114);
            // qvel = qvel_in[worldid, dofid]                                                     <L 453>
            var_115 = wp::address(var_qvel_in, var_0, var_98);
            var_117 = wp::load(var_115);
            var_116 = wp::copy(var_117);
            // Jqvel += j1mj2 * qvel                                                              <L 454>
            var_118 = wp::mul(var_103, var_116);
            var_119 = wp::add(var_91, var_118);
            // Jdotv += j1mj2_dot * qvel                                                          <L 455>
            var_120 = wp::mul(var_108, var_116);
            var_121 = wp::add(var_95, var_120);
            wp::assign(var_91, var_119);
            wp::assign(var_95, var_121);
            goto start_for_2;
        end_for_2:;
        // body_invweight0_id = worldid % body_invweight0.shape[0]                                <L 457>
        var_122 = &(var_body_invweight0.shape);
        var_125 = wp::load(var_122);
        var_124 = wp::extract(var_125, var_123);
        var_126 = wp::mod(var_0, var_124);
        // invweight = body_invweight0[body_invweight0_id, body1][0] + body_invweight0[body_invweight0_id, body2][0]       <L 458>
        var_127 = wp::address(var_body_invweight0, var_126, var_83);
        var_130 = wp::load(var_127);
        var_129 = wp::extract(var_130, var_128);
        var_131 = wp::address(var_body_invweight0, var_126, var_84);
        var_134 = wp::load(var_131);
        var_133 = wp::extract(var_134, var_132);
        var_135 = wp::add(var_129, var_133);
        // pos_imp = wp.length(pos)                                                               <L 459>
        var_136 = wp::length(var_87);
        // solref = eq_solref[worldid % eq_solref.shape[0], eqid]                                 <L 461>
        var_137 = &(var_eq_solref.shape);
        var_140 = wp::load(var_137);
        var_139 = wp::extract(var_140, var_138);
        var_141 = wp::mod(var_0, var_139);
        var_142 = wp::address(var_eq_solref, var_141, var_3);
        var_144 = wp::load(var_142);
        var_143 = wp::copy(var_144);
        // solimp = eq_solimp[worldid % eq_solimp.shape[0], eqid]                                 <L 462>
        var_145 = &(var_eq_solimp.shape);
        var_148 = wp::load(var_145);
        var_147 = wp::extract(var_148, var_146);
        var_149 = wp::mod(var_0, var_147);
        var_150 = wp::address(var_eq_solimp, var_149, var_3);
        var_152 = wp::load(var_150);
        var_151 = wp::copy(var_152);
        // timestep = opt_timestep[worldid % opt_timestep.shape[0]]                               <L 463>
        var_153 = &(var_opt_timestep.shape);
        var_156 = wp::load(var_153);
        var_155 = wp::extract(var_156, var_154);
        var_157 = wp::mod(var_0, var_155);
        var_158 = wp::address(var_opt_timestep, var_157);
        var_160 = wp::load(var_158);
        var_159 = wp::copy(var_160);
        // for i in range(3):                                                                     <L 465>
        // efcidi = efcid + i                                                                     <L 466>
        var_162 = wp::add(var_11, var_161);
        // _efc_row(                                                                              <L 468>
        // opt_disableflags,                                                                      <L 469>
        // worldid,                                                                               <L 470>
        // timestep,                                                                              <L 471>
        // efcidi,                                                                                <L 472>
        // pos[i],                                                                                <L 473>
        var_163 = wp::extract(var_87, var_161);
        // pos_imp,                                                                               <L 474>
        // invweight,                                                                             <L 475>
        // solref,                                                                                <L 476>
        // solimp,                                                                                <L 477>
        // 0.0,                                                                                   <L 478>
        // Jqvel[i],                                                                              <L 479>
        var_165 = wp::extract(var_91, var_161);
        // 0.0,                                                                                   <L 480>
        // ConstraintType.EQUALITY,                                                               <L 481>
        // eqid,                                                                                  <L 482>
        // efc_type_out,                                                                          <L 483>
        // efc_id_out,                                                                            <L 484>
        // efc_pos_out,                                                                           <L 485>
        // efc_margin_out,                                                                        <L 486>
        // efc_D_out,                                                                             <L 487>
        // efc_vel_out,                                                                           <L 488>
        // efc_aref_out,                                                                          <L 489>
        // efc_frictionloss_out,                                                                  <L 490>
        _efc_row_0(var_opt_disableflags, var_0, var_159, var_162, var_163, var_136, var_135, var_143, var_151, var_164, var_165, var_166, var_168, var_3, var_efc_type_out, var_efc_id_out, var_efc_pos_out, var_efc_margin_out, var_efc_D_out, var_efc_vel_out, var_efc_aref_out, var_efc_frictionloss_out);
        // efc_aref_out[worldid, efcidi] -= Jdotv[i]                                              <L 493>
        var_169 = wp::extract(var_95, var_161);
        var_170 = wp::atomic_sub(var_efc_aref_out, var_0, var_162, var_169);
        // efcidi = efcid + i                                                                     <L 466>
        var_172 = wp::add(var_11, var_171);
        // _efc_row(                                                                              <L 468>
        // opt_disableflags,                                                                      <L 469>
        // worldid,                                                                               <L 470>
        // timestep,                                                                              <L 471>
        // efcidi,                                                                                <L 472>
        // pos[i],                                                                                <L 473>
        var_173 = wp::extract(var_87, var_171);
        // pos_imp,                                                                               <L 474>
        // invweight,                                                                             <L 475>
        // solref,                                                                                <L 476>
        // solimp,                                                                                <L 477>
        // 0.0,                                                                                   <L 478>
        // Jqvel[i],                                                                              <L 479>
        var_175 = wp::extract(var_91, var_171);
        // 0.0,                                                                                   <L 480>
        // ConstraintType.EQUALITY,                                                               <L 481>
        // eqid,                                                                                  <L 482>
        // efc_type_out,                                                                          <L 483>
        // efc_id_out,                                                                            <L 484>
        // efc_pos_out,                                                                           <L 485>
        // efc_margin_out,                                                                        <L 486>
        // efc_D_out,                                                                             <L 487>
        // efc_vel_out,                                                                           <L 488>
        // efc_aref_out,                                                                          <L 489>
        // efc_frictionloss_out,                                                                  <L 490>
        _efc_row_0(var_opt_disableflags, var_0, var_159, var_172, var_173, var_136, var_135, var_143, var_151, var_174, var_175, var_176, var_178, var_3, var_efc_type_out, var_efc_id_out, var_efc_pos_out, var_efc_margin_out, var_efc_D_out, var_efc_vel_out, var_efc_aref_out, var_efc_frictionloss_out);
        // efc_aref_out[worldid, efcidi] -= Jdotv[i]                                              <L 493>
        var_179 = wp::extract(var_95, var_171);
        var_180 = wp::atomic_sub(var_efc_aref_out, var_0, var_172, var_179);
        // efcidi = efcid + i                                                                     <L 466>
        var_182 = wp::add(var_11, var_181);
        // _efc_row(                                                                              <L 468>
        // opt_disableflags,                                                                      <L 469>
        // worldid,                                                                               <L 470>
        // timestep,                                                                              <L 471>
        // efcidi,                                                                                <L 472>
        // pos[i],                                                                                <L 473>
        var_183 = wp::extract(var_87, var_181);
        // pos_imp,                                                                               <L 474>
        // invweight,                                                                             <L 475>
        // solref,                                                                                <L 476>
        // solimp,                                                                                <L 477>
        // 0.0,                                                                                   <L 478>
        // Jqvel[i],                                                                              <L 479>
        var_185 = wp::extract(var_91, var_181);
        // 0.0,                                                                                   <L 480>
        // ConstraintType.EQUALITY,                                                               <L 481>
        // eqid,                                                                                  <L 482>
        // efc_type_out,                                                                          <L 483>
        // efc_id_out,                                                                            <L 484>
        // efc_pos_out,                                                                           <L 485>
        // efc_margin_out,                                                                        <L 486>
        // efc_D_out,                                                                             <L 487>
        // efc_vel_out,                                                                           <L 488>
        // efc_aref_out,                                                                          <L 489>
        // efc_frictionloss_out,                                                                  <L 490>
        _efc_row_0(var_opt_disableflags, var_0, var_159, var_182, var_183, var_136, var_135, var_143, var_151, var_184, var_185, var_186, var_188, var_3, var_efc_type_out, var_efc_id_out, var_efc_pos_out, var_efc_margin_out, var_efc_D_out, var_efc_vel_out, var_efc_aref_out, var_efc_frictionloss_out);
        // efc_aref_out[worldid, efcidi] -= Jdotv[i]                                              <L 493>
        var_189 = wp::extract(var_95, var_181);
        var_190 = wp::atomic_sub(var_efc_aref_out, var_0, var_182, var_189);
    }
}


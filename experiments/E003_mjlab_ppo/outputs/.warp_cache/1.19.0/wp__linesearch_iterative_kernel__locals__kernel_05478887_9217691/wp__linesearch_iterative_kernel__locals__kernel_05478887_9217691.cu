#define WP_NO_BFLOAT16

#define WP_TILE_BLOCK_DIM 32
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


// /home/cofallen/miniconda3/envs/mjlab/lib/python3.12/site-packages/mujoco_warp/_src/solver.py:840
static CUDA_CALLABLE void _linesearch_iterative_kernel__locals___syncthreads_0(
    )
{
WP_TILE_SYNC();}


// /home/cofallen/miniconda3/envs/mjlab/lib/python3.12/site-packages/mujoco_warp/_src/solver.py:122
static CUDA_CALLABLE wp::vec_t<3, wp::float32> _eval_pt_direct_alpha_zero_0(
    wp::float32 var_jaref,
    wp::float32 var_jv,
    wp::float32 var_d)
{
    //---------
    // primal vars
    wp::float32 var_0;
    const wp::float32 var_1 = 0.5;
    wp::float32 var_2;
    wp::float32 var_3;
    wp::float32 var_4;
    wp::float32 var_5;
    wp::float32 var_6;
    wp::vec_t<3, wp::float32> var_7;
    //---------
    // forward
    // def _eval_pt_direct_alpha_zero(jaref: float, jv: float, d: float) -> wp.vec3:          <L 123>
    // jvD = jv * d                                                                           <L 125>
    var_0 = wp::mul(var_jv, var_d);
    // return wp.vec3(0.5 * d * jaref * jaref, jvD * jaref, jv * jvD)                         <L 126>
    var_2 = wp::mul(var_1, var_d);
    var_3 = wp::mul(var_2, var_jaref);
    var_4 = wp::mul(var_3, var_jaref);
    var_5 = wp::mul(var_0, var_jaref);
    var_6 = wp::mul(var_jv, var_0);
    var_7 = wp::vec_t<3, wp::float32>(var_4, var_5, var_6);
    return var_7;
}


// /home/cofallen/miniconda3/envs/mjlab/lib/python3.12/site-packages/mujoco_warp/_src/math.py:0
static CUDA_CALLABLE wp::float32 safe_div_0(
    wp::float32 var_x,
    wp::float32 var_y)
{
    //---------
    // primal vars
    const wp::float32 var_0 = 0.0;
    bool var_1;
    const wp::float32 var_2 = 1e-15;
    const wp::float32 var_3 = 1e-15;
    wp::float32 var_4;
    wp::float32 var_5;
    //---------
    // forward
    // def safe_div(x: Any, y: Any) -> Any:                                                   <L 1>
    // return x / wp.where(y != 0.0, y, types.MJ_MINVAL)                                      <L 2>
    var_1 = (var_y != var_0);
    var_4 = wp::where(var_1, var_y, var_3);
    var_5 = wp::div(var_x, var_4);
    return var_5;
}


// /home/cofallen/miniconda3/envs/mjlab/lib/python3.12/site-packages/mujoco_warp/_src/solver.py:223
static CUDA_CALLABLE wp::vec_t<3, wp::float32> _eval_frictionloss_pt_0(
    wp::float32 var_x,
    wp::float32 var_f,
    wp::float32 var_rf,
    wp::float32 var_jv,
    wp::float32 var_d)
{
    //---------
    // primal vars
    bool var_0;
    wp::float32 var_1;
    bool var_2;
    bool var_3;
    wp::float32 var_4;
    const wp::float32 var_5 = 0.5;
    wp::float32 var_6;
    wp::float32 var_7;
    wp::float32 var_8;
    wp::float32 var_9;
    wp::float32 var_10;
    wp::vec_t<3, wp::float32> var_11;
    wp::float32 var_12;
    bool var_13;
    const wp::float32 var_14 = -0.5;
    wp::float32 var_15;
    wp::float32 var_16;
    wp::float32 var_17;
    wp::float32 var_18;
    wp::float32 var_19;
    const wp::float32 var_20 = 0.0;
    wp::vec_t<3, wp::float32> var_21;
    const wp::float32 var_22 = -0.5;
    wp::float32 var_23;
    wp::float32 var_24;
    wp::float32 var_25;
    wp::float32 var_26;
    const wp::float32 var_27 = 0.0;
    wp::vec_t<3, wp::float32> var_28;
    //---------
    // forward
    // def _eval_frictionloss_pt(x: float, f: float, rf: float, jv: float, d: float) -> wp.vec3:       <L 224>
    // if (-rf < x) and (x < rf):                                                             <L 226>
    var_1 = wp::neg(var_rf);
    var_2 = (var_1 < var_x);
    var_0 = var_2;
    if (var_0) {
        var_3 = (var_x < var_rf);
        var_0 = var_0 && var_3;
    }
    if (var_0) {
        // jvD = jv * d                                                                       <L 227>
        var_4 = wp::mul(var_jv, var_d);
        // return wp.vec3(0.5 * d * x * x, jvD * x, jv * jvD)                                 <L 228>
        var_6 = wp::mul(var_5, var_d);
        var_7 = wp::mul(var_6, var_x);
        var_8 = wp::mul(var_7, var_x);
        var_9 = wp::mul(var_4, var_x);
        var_10 = wp::mul(var_jv, var_4);
        var_11 = wp::vec_t<3, wp::float32>(var_8, var_9, var_10);
        return var_11;
    }
    if (!var_0) {
        // elif x <= -rf:                                                                     <L 229>
        var_12 = wp::neg(var_rf);
        var_13 = (var_x <= var_12);
        if (var_13) {
            // return wp.vec3(f * (-0.5 * rf - x), -f * jv, 0.0)                              <L 230>
            var_15 = wp::mul(var_14, var_rf);
            var_16 = wp::sub(var_15, var_x);
            var_17 = wp::mul(var_f, var_16);
            var_18 = wp::neg(var_f);
            var_19 = wp::mul(var_18, var_jv);
            var_21 = wp::vec_t<3, wp::float32>(var_17, var_19, var_20);
            return var_21;
        }
        if (!var_13) {
            // return wp.vec3(f * (-0.5 * rf + x), f * jv, 0.0)                               <L 232>
            var_23 = wp::mul(var_22, var_rf);
            var_24 = wp::add(var_23, var_x);
            var_25 = wp::mul(var_f, var_24);
            var_26 = wp::mul(var_f, var_jv);
            var_28 = wp::vec_t<3, wp::float32>(var_25, var_26, var_27);
            return var_28;
        }
    }
    return {};
}


// /home/cofallen/miniconda3/envs/mjlab/lib/python3.12/site-packages/mujoco_warp/_src/solver.py:562
static CUDA_CALLABLE wp::vec_t<3, wp::float32> _compute_efc_eval_pt_alpha_zero_pyramidal_0(
    wp::int32 var_efcid,
    wp::int32 var_ne,
    wp::int32 var_nf,
    wp::float32 var_efc_D,
    wp::array_t<wp::float32> var_efc_frictionloss,
    wp::float32 var_ctx_Jaref,
    wp::float32 var_ctx_jv)
{
    //---------
    // primal vars
    wp::int32 var_0;
    bool var_1;
    const wp::float32 var_2 = 0.0;
    bool var_3;
    wp::vec_t<3, wp::float32> var_4;
    const wp::float32 var_5 = 0.0;
    wp::vec_t<3, wp::float32> var_6;
    bool var_7;
    wp::float32* var_8;
    wp::float32 var_9;
    wp::float32 var_10;
    wp::float32 var_11;
    wp::vec_t<3, wp::float32> var_12;
    wp::vec_t<3, wp::float32> var_13;
    //---------
    // forward
    // def _compute_efc_eval_pt_alpha_zero_pyramidal(                                         <L 563>
    // if efcid >= ne + nf:                                                                   <L 575>
    var_0 = wp::add(var_ne, var_nf);
    var_1 = (var_efcid >= var_0);
    if (var_1) {
        // if ctx_Jaref < 0.0:                                                                <L 576>
        var_3 = (var_ctx_Jaref < var_2);
        if (var_3) {
            // return _eval_pt_direct_alpha_zero(ctx_Jaref, ctx_jv, efc_D)                    <L 577>
            var_4 = _eval_pt_direct_alpha_zero_0(var_ctx_Jaref, var_ctx_jv, var_efc_D);
            return var_4;
        }
        // return wp.vec3(0.0)                                                                <L 578>
        var_6 = wp::vec_t<3, wp::float32>(var_5);
        return var_6;
    }
    // if efcid >= ne:                                                                        <L 581>
    var_7 = (var_efcid >= var_ne);
    if (var_7) {
        // f = efc_frictionloss[efcid]                                                        <L 582>
        var_8 = wp::address(var_efc_frictionloss, var_efcid);
        var_10 = wp::load(var_8);
        var_9 = wp::copy(var_10);
        // rf = math.safe_div(f, efc_D)                                                       <L 583>
        var_11 = safe_div_0(var_9, var_efc_D);
        // return _eval_frictionloss_pt(ctx_Jaref, f, rf, ctx_jv, efc_D)                      <L 584>
        var_12 = _eval_frictionloss_pt_0(var_ctx_Jaref, var_9, var_11, var_ctx_jv, var_efc_D);
        return var_12;
    }
    // return _eval_pt_direct_alpha_zero(ctx_Jaref, ctx_jv, efc_D)                            <L 587>
    var_13 = _eval_pt_direct_alpha_zero_0(var_ctx_Jaref, var_ctx_jv, var_efc_D);
    return var_13;
}


// /home/cofallen/miniconda3/envs/mjlab/lib/python3.12/site-packages/mujoco_warp/_src/solver.py:137
static CUDA_CALLABLE wp::float32 _eval_pt_direct_cost_alpha_zero_0(
    wp::float32 var_jaref,
    wp::float32 var_d)
{
    //---------
    // primal vars
    const wp::float32 var_0 = 0.5;
    wp::float32 var_1;
    wp::float32 var_2;
    wp::float32 var_3;
    //---------
    // forward
    // def _eval_pt_direct_cost_alpha_zero(jaref: float, d: float) -> float:                  <L 138>
    // return 0.5 * d * jaref * jaref                                                         <L 139>
    var_1 = wp::mul(var_0, var_d);
    var_2 = wp::mul(var_1, var_jaref);
    var_3 = wp::mul(var_2, var_jaref);
    return var_3;
}


// /home/cofallen/miniconda3/envs/mjlab/lib/python3.12/site-packages/mujoco_warp/_src/solver.py:142
static CUDA_CALLABLE wp::vec_t<3, wp::float32> _eval_pt_direct_shifted_0(
    wp::float32 var_jaref,
    wp::float32 var_jv,
    wp::float32 var_d,
    wp::float32 var_alpha,
    wp::float32 var_offset)
{
    //---------
    // primal vars
    wp::float32 var_0;
    wp::float32 var_1;
    wp::float32 var_2;
    wp::float32 var_3;
    const wp::float32 var_4 = 0.5;
    wp::float32 var_5;
    wp::float32 var_6;
    wp::float32 var_7;
    wp::float32 var_8;
    wp::float32 var_9;
    wp::float32 var_10;
    wp::vec_t<3, wp::float32> var_11;
    //---------
    // forward
    // def _eval_pt_direct_shifted(jaref: float, jv: float, d: float, alpha: float, offset: float) -> wp.vec3:       <L 143>
    // jvD = jv * d                                                                           <L 145>
    var_0 = wp::mul(var_jv, var_d);
    // hessian = jv * jvD                                                                     <L 146>
    var_1 = wp::mul(var_jv, var_0);
    // alpha_h = alpha * hessian                                                              <L 147>
    var_2 = wp::mul(var_alpha, var_1);
    // return wp.vec3(alpha * (jvD * jaref + 0.5 * alpha_h) + offset, jvD * jaref + alpha_h, hessian)       <L 148>
    var_3 = wp::mul(var_0, var_jaref);
    var_5 = wp::mul(var_4, var_2);
    var_6 = wp::add(var_3, var_5);
    var_7 = wp::mul(var_alpha, var_6);
    var_8 = wp::add(var_7, var_offset);
    var_9 = wp::mul(var_0, var_jaref);
    var_10 = wp::add(var_9, var_2);
    var_11 = wp::vec_t<3, wp::float32>(var_8, var_10, var_1);
    return var_11;
}


// /home/cofallen/miniconda3/envs/mjlab/lib/python3.12/site-packages/mujoco_warp/_src/solver.py:235
static CUDA_CALLABLE wp::float32 _eval_frictionloss_cost_0(
    wp::float32 var_x,
    wp::float32 var_f,
    wp::float32 var_rf,
    wp::float32 var_d)
{
    //---------
    // primal vars
    bool var_0;
    wp::float32 var_1;
    bool var_2;
    bool var_3;
    const wp::float32 var_4 = 0.5;
    wp::float32 var_5;
    wp::float32 var_6;
    wp::float32 var_7;
    wp::float32 var_8;
    bool var_9;
    const wp::float32 var_10 = -0.5;
    wp::float32 var_11;
    wp::float32 var_12;
    wp::float32 var_13;
    const wp::float32 var_14 = -0.5;
    wp::float32 var_15;
    wp::float32 var_16;
    wp::float32 var_17;
    //---------
    // forward
    // def _eval_frictionloss_cost(x: float, f: float, rf: float, d: float) -> float:         <L 236>
    // if (-rf < x) and (x < rf):                                                             <L 237>
    var_1 = wp::neg(var_rf);
    var_2 = (var_1 < var_x);
    var_0 = var_2;
    if (var_0) {
        var_3 = (var_x < var_rf);
        var_0 = var_0 && var_3;
    }
    if (var_0) {
        // return 0.5 * d * x * x                                                             <L 238>
        var_5 = wp::mul(var_4, var_d);
        var_6 = wp::mul(var_5, var_x);
        var_7 = wp::mul(var_6, var_x);
        return var_7;
    }
    if (!var_0) {
        // elif x <= -rf:                                                                     <L 239>
        var_8 = wp::neg(var_rf);
        var_9 = (var_x <= var_8);
        if (var_9) {
            // return f * (-0.5 * rf - x)                                                     <L 240>
            var_11 = wp::mul(var_10, var_rf);
            var_12 = wp::sub(var_11, var_x);
            var_13 = wp::mul(var_f, var_12);
            return var_13;
        }
    }
    // return f * (-0.5 * rf + x)                                                             <L 241>
    var_15 = wp::mul(var_14, var_rf);
    var_16 = wp::add(var_15, var_x);
    var_17 = wp::mul(var_f, var_16);
    return var_17;
}


// /home/cofallen/miniconda3/envs/mjlab/lib/python3.12/site-packages/mujoco_warp/_src/solver.py:218
static CUDA_CALLABLE wp::vec_t<3, wp::float32> _shift_cost_0(
    wp::vec_t<3, wp::float32> var_pt,
    wp::float32 var_cost0)
{
    //---------
    // primal vars
    const wp::int32 var_0 = 0;
    wp::float32 var_1;
    wp::float32 var_2;
    const wp::int32 var_3 = 1;
    wp::float32 var_4;
    const wp::int32 var_5 = 2;
    wp::float32 var_6;
    wp::vec_t<3, wp::float32> var_7;
    //---------
    // forward
    // def _shift_cost(pt: wp.vec3, cost0: float) -> wp.vec3:                                 <L 219>
    // return wp.vec3(pt[0] - cost0, pt[1], pt[2])                                            <L 220>
    var_1 = wp::extract(var_pt, var_0);
    var_2 = wp::sub(var_1, var_cost0);
    var_4 = wp::extract(var_pt, var_3);
    var_6 = wp::extract(var_pt, var_5);
    var_7 = wp::vec_t<3, wp::float32>(var_2, var_4, var_6);
    return var_7;
}


// /home/cofallen/miniconda3/envs/mjlab/lib/python3.12/site-packages/mujoco_warp/_src/solver.py:466
static CUDA_CALLABLE wp::vec_t<3, wp::float32> _compute_efc_eval_pt_pyramidal_0(
    wp::int32 var_efcid,
    wp::float32 var_alpha,
    wp::int32 var_ne,
    wp::int32 var_nf,
    wp::float32 var_efc_D,
    wp::array_t<wp::float32> var_efc_frictionloss,
    wp::float32 var_ctx_Jaref,
    wp::float32 var_ctx_jv)
{
    //---------
    // primal vars
    wp::int32 var_0;
    bool var_1;
    wp::float32 var_2;
    wp::float32 var_3;
    wp::float32 var_4;
    const wp::float32 var_5 = 0.0;
    bool var_6;
    const wp::float32 var_7 = 0.0;
    wp::float32 var_8;
    wp::float32 var_9;
    const wp::float32 var_10 = 0.0;
    bool var_11;
    wp::vec_t<3, wp::float32> var_12;
    wp::float32 var_13;
    const wp::float32 var_14 = 0.0;
    const wp::float32 var_15 = 0.0;
    wp::vec_t<3, wp::float32> var_16;
    bool var_17;
    wp::float32* var_18;
    wp::float32 var_19;
    wp::float32 var_20;
    wp::float32 var_21;
    wp::float32 var_22;
    wp::float32 var_23;
    wp::vec_t<3, wp::float32> var_24;
    wp::float32 var_25;
    wp::vec_t<3, wp::float32> var_26;
    wp::float32 var_27;
    const wp::float32 var_28 = 0.0;
    wp::vec_t<3, wp::float32> var_29;
    //---------
    // forward
    // def _compute_efc_eval_pt_pyramidal(                                                    <L 467>
    // if efcid >= ne + nf:                                                                   <L 483>
    var_0 = wp::add(var_ne, var_nf);
    var_1 = (var_efcid >= var_0);
    if (var_1) {
        // x = ctx_Jaref + alpha * ctx_jv                                                     <L 484>
        var_2 = wp::mul(var_alpha, var_ctx_jv);
        var_3 = wp::add(var_ctx_Jaref, var_2);
        // quad0 = _eval_pt_direct_cost_alpha_zero(ctx_Jaref, efc_D)                          <L 485>
        var_4 = _eval_pt_direct_cost_alpha_zero_0(var_ctx_Jaref, var_efc_D);
        // cost0 = wp.where(ctx_Jaref < 0.0, quad0, 0.0)                                      <L 486>
        var_6 = (var_ctx_Jaref < var_5);
        var_8 = wp::where(var_6, var_4, var_7);
        // offset = quad0 - cost0                                                             <L 489>
        var_9 = wp::sub(var_4, var_8);
        // if x < 0.0:                                                                        <L 490>
        var_11 = (var_3 < var_10);
        if (var_11) {
            // return _eval_pt_direct_shifted(ctx_Jaref, ctx_jv, efc_D, alpha, offset)        <L 491>
            var_12 = _eval_pt_direct_shifted_0(var_ctx_Jaref, var_ctx_jv, var_efc_D, var_alpha, var_9);
            return var_12;
        }
        // return wp.vec3(-cost0, 0.0, 0.0)                                                   <L 492>
        var_13 = wp::neg(var_8);
        var_16 = wp::vec_t<3, wp::float32>(var_13, var_14, var_15);
        return var_16;
    }
    // if efcid >= ne:                                                                        <L 495>
    var_17 = (var_efcid >= var_ne);
    if (var_17) {
        // f = efc_frictionloss[efcid]                                                        <L 496>
        var_18 = wp::address(var_efc_frictionloss, var_efcid);
        var_20 = wp::load(var_18);
        var_19 = wp::copy(var_20);
        // x = ctx_Jaref + alpha * ctx_jv                                                     <L 497>
        var_21 = wp::mul(var_alpha, var_ctx_jv);
        var_22 = wp::add(var_ctx_Jaref, var_21);
        // rf = math.safe_div(f, efc_D)                                                       <L 498>
        var_23 = safe_div_0(var_19, var_efc_D);
        // return _shift_cost(_eval_frictionloss_pt(x, f, rf, ctx_jv, efc_D), _eval_frictionloss_cost(ctx_Jaref, f, rf, efc_D))       <L 499>
        var_24 = _eval_frictionloss_pt_0(var_22, var_19, var_23, var_ctx_jv, var_efc_D);
        var_25 = _eval_frictionloss_cost_0(var_ctx_Jaref, var_19, var_23, var_efc_D);
        var_26 = _shift_cost_0(var_24, var_25);
        return var_26;
    }
    var_27 = wp::where(var_17, var_22, var_3);
    // return _eval_pt_direct_shifted(ctx_Jaref, ctx_jv, efc_D, alpha, 0.0)                   <L 502>
    var_29 = _eval_pt_direct_shifted_0(var_ctx_Jaref, var_ctx_jv, var_efc_D, var_alpha, var_28);
    return var_29;
}


// /home/cofallen/miniconda3/envs/mjlab/lib/python3.12/site-packages/mujoco_warp/_src/solver.py:192
static CUDA_CALLABLE wp::vec_t<3, wp::float32> _eval_pt_0(
    wp::vec_t<3, wp::float32> var_quad,
    wp::float32 var_alpha)
{
    //---------
    // primal vars
    const wp::int32 var_0 = 2;
    wp::float32 var_1;
    wp::float32 var_2;
    wp::float32 var_3;
    const wp::int32 var_4 = 1;
    wp::float32 var_5;
    wp::float32 var_6;
    wp::float32 var_7;
    const wp::int32 var_8 = 0;
    wp::float32 var_9;
    wp::float32 var_10;
    const wp::float32 var_11 = 2.0;
    wp::float32 var_12;
    const wp::int32 var_13 = 1;
    wp::float32 var_14;
    wp::float32 var_15;
    const wp::float32 var_16 = 2.0;
    const wp::int32 var_17 = 2;
    wp::float32 var_18;
    wp::float32 var_19;
    wp::vec_t<3, wp::float32> var_20;
    //---------
    // forward
    // def _eval_pt(quad: wp.vec3, alpha: float) -> wp.vec3:                                  <L 193>
    // aq2 = alpha * quad[2]                                                                  <L 195>
    var_1 = wp::extract(var_quad, var_0);
    var_2 = wp::mul(var_alpha, var_1);
    // return wp.vec3(                                                                        <L 196>
    // alpha * aq2 + alpha * quad[1] + quad[0],                                               <L 197>
    var_3 = wp::mul(var_alpha, var_2);
    var_5 = wp::extract(var_quad, var_4);
    var_6 = wp::mul(var_alpha, var_5);
    var_7 = wp::add(var_3, var_6);
    var_9 = wp::extract(var_quad, var_8);
    var_10 = wp::add(var_7, var_9);
    // 2.0 * aq2 + quad[1],                                                                   <L 198>
    var_12 = wp::mul(var_11, var_2);
    var_14 = wp::extract(var_quad, var_13);
    var_15 = wp::add(var_12, var_14);
    // 2.0 * quad[2],                                                                         <L 199>
    var_18 = wp::extract(var_quad, var_17);
    var_19 = wp::mul(var_16, var_18);
    var_20 = wp::vec_t<3, wp::float32>(var_10, var_15, var_19);
    return var_20;
}


// /home/cofallen/miniconda3/envs/mjlab/lib/python3.12/site-packages/mujoco_warp/_src/solver.py:169
static CUDA_CALLABLE void _eval_pt_direct_shifted_3alphas_0(
    wp::float32 var_jaref,
    wp::float32 var_jv,
    wp::float32 var_d,
    wp::float32 var_lo_alpha,
    wp::float32 var_hi_alpha,
    wp::float32 var_mid_alpha,
    wp::float32 var_offset,
    wp::vec_t<3, wp::float32> & ret_0,
    wp::vec_t<3, wp::float32> & ret_1,
    wp::vec_t<3, wp::float32> & ret_2)
{
    //---------
    // primal vars
    wp::float32 var_0;
    wp::float32 var_1;
    wp::float32 var_2;
    wp::float32 var_3;
    wp::float32 var_4;
    wp::float32 var_5;
    const wp::float32 var_6 = 0.5;
    wp::float32 var_7;
    wp::float32 var_8;
    wp::float32 var_9;
    wp::float32 var_10;
    wp::float32 var_11;
    wp::vec_t<3, wp::float32> var_12;
    const wp::float32 var_13 = 0.5;
    wp::float32 var_14;
    wp::float32 var_15;
    wp::float32 var_16;
    wp::float32 var_17;
    wp::float32 var_18;
    wp::vec_t<3, wp::float32> var_19;
    const wp::float32 var_20 = 0.5;
    wp::float32 var_21;
    wp::float32 var_22;
    wp::float32 var_23;
    wp::float32 var_24;
    wp::float32 var_25;
    wp::vec_t<3, wp::float32> var_26;
    //---------
    // forward
    // def _eval_pt_direct_shifted_3alphas(                                                   <L 170>
    // jvD = jv * d                                                                           <L 174>
    var_0 = wp::mul(var_jv, var_d);
    // grad0 = jvD * jaref                                                                    <L 175>
    var_1 = wp::mul(var_0, var_jaref);
    // hessian = jv * jvD                                                                     <L 176>
    var_2 = wp::mul(var_jv, var_0);
    // lo_ah = lo_alpha * hessian                                                             <L 177>
    var_3 = wp::mul(var_lo_alpha, var_2);
    // hi_ah = hi_alpha * hessian                                                             <L 178>
    var_4 = wp::mul(var_hi_alpha, var_2);
    // mid_ah = mid_alpha * hessian                                                           <L 179>
    var_5 = wp::mul(var_mid_alpha, var_2);
    // return (                                                                               <L 180>
    // wp.vec3(lo_alpha * (grad0 + 0.5 * lo_ah) + offset, grad0 + lo_ah, hessian),            <L 181>
    var_7 = wp::mul(var_6, var_3);
    var_8 = wp::add(var_1, var_7);
    var_9 = wp::mul(var_lo_alpha, var_8);
    var_10 = wp::add(var_9, var_offset);
    var_11 = wp::add(var_1, var_3);
    var_12 = wp::vec_t<3, wp::float32>(var_10, var_11, var_2);
    // wp.vec3(hi_alpha * (grad0 + 0.5 * hi_ah) + offset, grad0 + hi_ah, hessian),            <L 182>
    var_14 = wp::mul(var_13, var_4);
    var_15 = wp::add(var_1, var_14);
    var_16 = wp::mul(var_hi_alpha, var_15);
    var_17 = wp::add(var_16, var_offset);
    var_18 = wp::add(var_1, var_4);
    var_19 = wp::vec_t<3, wp::float32>(var_17, var_18, var_2);
    // wp.vec3(mid_alpha * (grad0 + 0.5 * mid_ah) + offset, grad0 + mid_ah, hessian),         <L 183>
    var_21 = wp::mul(var_20, var_5);
    var_22 = wp::add(var_1, var_21);
    var_23 = wp::mul(var_mid_alpha, var_22);
    var_24 = wp::add(var_23, var_offset);
    var_25 = wp::add(var_1, var_5);
    var_26 = wp::vec_t<3, wp::float32>(var_24, var_25, var_2);
    ret_0 = var_12;
    ret_1 = var_19;
    ret_2 = var_26;
    return;
}


// /home/cofallen/miniconda3/envs/mjlab/lib/python3.12/site-packages/mujoco_warp/_src/solver.py:244
static CUDA_CALLABLE wp::vec_t<3, wp::float32> _eval_frictionloss_pt_one_0(
    wp::float32 var_x,
    wp::float32 var_f,
    wp::float32 var_rf,
    wp::float32 var_half_d,
    wp::float32 var_jvD,
    wp::float32 var_hessian,
    wp::float32 var_f_jv)
{
    //---------
    // primal vars
    bool var_0;
    wp::float32 var_1;
    bool var_2;
    bool var_3;
    wp::float32 var_4;
    wp::float32 var_5;
    wp::float32 var_6;
    wp::vec_t<3, wp::float32> var_7;
    wp::float32 var_8;
    bool var_9;
    const wp::float32 var_10 = -0.5;
    wp::float32 var_11;
    wp::float32 var_12;
    wp::float32 var_13;
    wp::float32 var_14;
    const wp::float32 var_15 = 0.0;
    wp::vec_t<3, wp::float32> var_16;
    const wp::float32 var_17 = -0.5;
    wp::float32 var_18;
    wp::float32 var_19;
    wp::float32 var_20;
    const wp::float32 var_21 = 0.0;
    wp::vec_t<3, wp::float32> var_22;
    //---------
    // forward
    // def _eval_frictionloss_pt_one(x: float, f: float, rf: float, half_d: float, jvD: float, hessian: float, f_jv: float) -> wp.vec3:       <L 245>
    // if (-rf < x) and (x < rf):                                                             <L 247>
    var_1 = wp::neg(var_rf);
    var_2 = (var_1 < var_x);
    var_0 = var_2;
    if (var_0) {
        var_3 = (var_x < var_rf);
        var_0 = var_0 && var_3;
    }
    if (var_0) {
        // return wp.vec3(half_d * x * x, jvD * x, hessian)                                   <L 248>
        var_4 = wp::mul(var_half_d, var_x);
        var_5 = wp::mul(var_4, var_x);
        var_6 = wp::mul(var_jvD, var_x);
        var_7 = wp::vec_t<3, wp::float32>(var_5, var_6, var_hessian);
        return var_7;
    }
    if (!var_0) {
        // elif x <= -rf:                                                                     <L 249>
        var_8 = wp::neg(var_rf);
        var_9 = (var_x <= var_8);
        if (var_9) {
            // return wp.vec3(f * (-0.5 * rf - x), -f_jv, 0.0)                                <L 250>
            var_11 = wp::mul(var_10, var_rf);
            var_12 = wp::sub(var_11, var_x);
            var_13 = wp::mul(var_f, var_12);
            var_14 = wp::neg(var_f_jv);
            var_16 = wp::vec_t<3, wp::float32>(var_13, var_14, var_15);
            return var_16;
        }
        if (!var_9) {
            // return wp.vec3(f * (-0.5 * rf + x), f_jv, 0.0)                                 <L 252>
            var_18 = wp::mul(var_17, var_rf);
            var_19 = wp::add(var_18, var_x);
            var_20 = wp::mul(var_f, var_19);
            var_22 = wp::vec_t<3, wp::float32>(var_20, var_f_jv, var_21);
            return var_22;
        }
    }
    return {};
}


// /home/cofallen/miniconda3/envs/mjlab/lib/python3.12/site-packages/mujoco_warp/_src/solver.py:255
static CUDA_CALLABLE void _eval_frictionloss_pt_3alphas_0(
    wp::float32 var_x_lo,
    wp::float32 var_x_hi,
    wp::float32 var_x_mid,
    wp::float32 var_f,
    wp::float32 var_rf,
    wp::float32 var_jv,
    wp::float32 var_d,
    wp::vec_t<3, wp::float32> & ret_0,
    wp::vec_t<3, wp::float32> & ret_1,
    wp::vec_t<3, wp::float32> & ret_2)
{
    //---------
    // primal vars
    wp::float32 var_0;
    const wp::float32 var_1 = 0.5;
    wp::float32 var_2;
    wp::float32 var_3;
    wp::float32 var_4;
    wp::vec_t<3, wp::float32> var_5;
    wp::vec_t<3, wp::float32> var_6;
    wp::vec_t<3, wp::float32> var_7;
    //---------
    // forward
    // def _eval_frictionloss_pt_3alphas(                                                     <L 256>
    // jvD = jv * d                                                                           <L 260>
    var_0 = wp::mul(var_jv, var_d);
    // half_d = 0.5 * d                                                                       <L 261>
    var_2 = wp::mul(var_1, var_d);
    // hessian = jv * jvD                                                                     <L 262>
    var_3 = wp::mul(var_jv, var_0);
    // f_jv = f * jv                                                                          <L 263>
    var_4 = wp::mul(var_f, var_jv);
    // return (                                                                               <L 264>
    // _eval_frictionloss_pt_one(x_lo, f, rf, half_d, jvD, hessian, f_jv),                    <L 265>
    var_5 = _eval_frictionloss_pt_one_0(var_x_lo, var_f, var_rf, var_2, var_0, var_3, var_4);
    // _eval_frictionloss_pt_one(x_hi, f, rf, half_d, jvD, hessian, f_jv),                    <L 266>
    var_6 = _eval_frictionloss_pt_one_0(var_x_hi, var_f, var_rf, var_2, var_0, var_3, var_4);
    // _eval_frictionloss_pt_one(x_mid, f, rf, half_d, jvD, hessian, f_jv),                   <L 267>
    var_7 = _eval_frictionloss_pt_one_0(var_x_mid, var_f, var_rf, var_2, var_0, var_3, var_4);
    ret_0 = var_5;
    ret_1 = var_6;
    ret_2 = var_7;
    return;
}


// /home/cofallen/miniconda3/envs/mjlab/lib/python3.12/site-packages/mujoco_warp/_src/solver.py:634
static CUDA_CALLABLE void _compute_efc_eval_pt_3alphas_pyramidal_0(
    wp::int32 var_efcid,
    wp::float32 var_lo_alpha,
    wp::float32 var_hi_alpha,
    wp::float32 var_mid_alpha,
    wp::int32 var_ne,
    wp::int32 var_nf,
    wp::float32 var_efc_D,
    wp::array_t<wp::float32> var_efc_frictionloss,
    wp::float32 var_ctx_Jaref,
    wp::float32 var_ctx_jv,
    wp::vec_t<3, wp::float32> & ret_0,
    wp::vec_t<3, wp::float32> & ret_1,
    wp::vec_t<3, wp::float32> & ret_2)
{
    //---------
    // primal vars
    wp::int32 var_0;
    bool var_1;
    wp::float32 var_2;
    wp::float32 var_3;
    wp::float32 var_4;
    wp::float32 var_5;
    wp::float32 var_6;
    wp::float32 var_7;
    wp::float32 var_8;
    const wp::float32 var_9 = 0.0;
    bool var_10;
    const wp::float32 var_11 = 0.0;
    wp::float32 var_12;
    wp::float32 var_13;
    wp::vec_t<3, wp::float32> var_14;
    wp::vec_t<3, wp::float32> var_15;
    wp::vec_t<3, wp::float32> var_16;
    wp::float32 var_17;
    const wp::float32 var_18 = 0.0;
    const wp::float32 var_19 = 0.0;
    wp::vec_t<3, wp::float32> var_20;
    const wp::float32 var_21 = 0.0;
    bool var_22;
    wp::vec_t<3, wp::float32> var_23;
    const wp::float32 var_24 = 0.0;
    bool var_25;
    wp::vec_t<3, wp::float32> var_26;
    const wp::float32 var_27 = 0.0;
    bool var_28;
    wp::vec_t<3, wp::float32> var_29;
    bool var_30;
    wp::float32 var_31;
    wp::float32 var_32;
    wp::float32 var_33;
    wp::float32 var_34;
    wp::float32 var_35;
    wp::float32 var_36;
    wp::float32* var_37;
    wp::float32 var_38;
    wp::float32 var_39;
    wp::float32 var_40;
    wp::float32 var_41;
    wp::vec_t<3, wp::float32> var_42;
    wp::vec_t<3, wp::float32> var_43;
    wp::vec_t<3, wp::float32> var_44;
    wp::vec_t<3, wp::float32> var_45;
    wp::vec_t<3, wp::float32> var_46;
    wp::vec_t<3, wp::float32> var_47;
    wp::float32 var_48;
    wp::float32 var_49;
    wp::float32 var_50;
    wp::float32 var_51;
    const wp::float32 var_52 = 0.0;
    wp::vec_t<3, wp::float32> var_53;
    wp::vec_t<3, wp::float32> var_54;
    wp::vec_t<3, wp::float32> var_55;
    //---------
    // forward
    // def _compute_efc_eval_pt_3alphas_pyramidal(                                            <L 635>
    // if efcid >= ne + nf:                                                                   <L 654>
    var_0 = wp::add(var_ne, var_nf);
    var_1 = (var_efcid >= var_0);
    if (var_1) {
        // x_lo = ctx_Jaref + lo_alpha * ctx_jv                                               <L 655>
        var_2 = wp::mul(var_lo_alpha, var_ctx_jv);
        var_3 = wp::add(var_ctx_Jaref, var_2);
        // x_hi = ctx_Jaref + hi_alpha * ctx_jv                                               <L 656>
        var_4 = wp::mul(var_hi_alpha, var_ctx_jv);
        var_5 = wp::add(var_ctx_Jaref, var_4);
        // x_mid = ctx_Jaref + mid_alpha * ctx_jv                                             <L 657>
        var_6 = wp::mul(var_mid_alpha, var_ctx_jv);
        var_7 = wp::add(var_ctx_Jaref, var_6);
        // quad0 = _eval_pt_direct_cost_alpha_zero(ctx_Jaref, efc_D)                          <L 658>
        var_8 = _eval_pt_direct_cost_alpha_zero_0(var_ctx_Jaref, var_efc_D);
        // cost0 = wp.where(ctx_Jaref < 0.0, quad0, 0.0)                                      <L 659>
        var_10 = (var_ctx_Jaref < var_9);
        var_12 = wp::where(var_10, var_8, var_11);
        // offset = quad0 - cost0                                                             <L 661>
        var_13 = wp::sub(var_8, var_12);
        // pt_lo, pt_hi, pt_mid = _eval_pt_direct_shifted_3alphas(ctx_Jaref, ctx_jv, efc_D, lo_alpha, hi_alpha, mid_alpha, offset)       <L 662>
        _eval_pt_direct_shifted_3alphas_0(var_ctx_Jaref, var_ctx_jv, var_efc_D, var_lo_alpha, var_hi_alpha, var_mid_alpha, var_13, var_14, var_15, var_16);
        // inactive = wp.vec3(-cost0, 0.0, 0.0)                                               <L 663>
        var_17 = wp::neg(var_12);
        var_20 = wp::vec_t<3, wp::float32>(var_17, var_18, var_19);
        // return (                                                                           <L 664>
        // wp.where(x_lo < 0.0, pt_lo, inactive),                                             <L 665>
        var_22 = (var_3 < var_21);
        var_23 = wp::where(var_22, var_14, var_20);
        // wp.where(x_hi < 0.0, pt_hi, inactive),                                             <L 666>
        var_25 = (var_5 < var_24);
        var_26 = wp::where(var_25, var_15, var_20);
        // wp.where(x_mid < 0.0, pt_mid, inactive),                                           <L 667>
        var_28 = (var_7 < var_27);
        var_29 = wp::where(var_28, var_16, var_20);
        ret_0 = var_23;
        ret_1 = var_26;
        ret_2 = var_29;
        return;
    }
    // if efcid >= ne:                                                                        <L 671>
    var_30 = (var_efcid >= var_ne);
    if (var_30) {
        // x_lo = ctx_Jaref + lo_alpha * ctx_jv                                               <L 672>
        var_31 = wp::mul(var_lo_alpha, var_ctx_jv);
        var_32 = wp::add(var_ctx_Jaref, var_31);
        // x_hi = ctx_Jaref + hi_alpha * ctx_jv                                               <L 673>
        var_33 = wp::mul(var_hi_alpha, var_ctx_jv);
        var_34 = wp::add(var_ctx_Jaref, var_33);
        // x_mid = ctx_Jaref + mid_alpha * ctx_jv                                             <L 674>
        var_35 = wp::mul(var_mid_alpha, var_ctx_jv);
        var_36 = wp::add(var_ctx_Jaref, var_35);
        // f = efc_frictionloss[efcid]                                                        <L 675>
        var_37 = wp::address(var_efc_frictionloss, var_efcid);
        var_39 = wp::load(var_37);
        var_38 = wp::copy(var_39);
        // rf = math.safe_div(f, efc_D)                                                       <L 676>
        var_40 = safe_div_0(var_38, var_efc_D);
        // cost0 = _eval_frictionloss_cost(ctx_Jaref, f, rf, efc_D)                           <L 677>
        var_41 = _eval_frictionloss_cost_0(var_ctx_Jaref, var_38, var_40, var_efc_D);
        // lo, hi, mid = _eval_frictionloss_pt_3alphas(x_lo, x_hi, x_mid, f, rf, ctx_jv, efc_D)       <L 678>
        _eval_frictionloss_pt_3alphas_0(var_32, var_34, var_36, var_38, var_40, var_ctx_jv, var_efc_D, var_42, var_43, var_44);
        // return (_shift_cost(lo, cost0), _shift_cost(hi, cost0), _shift_cost(mid, cost0))       <L 679>
        var_45 = _shift_cost_0(var_42, var_41);
        var_46 = _shift_cost_0(var_43, var_41);
        var_47 = _shift_cost_0(var_44, var_41);
        ret_0 = var_45;
        ret_1 = var_46;
        ret_2 = var_47;
        return;
    }
    var_48 = wp::where(var_30, var_32, var_3);
    var_49 = wp::where(var_30, var_34, var_5);
    var_50 = wp::where(var_30, var_36, var_7);
    var_51 = wp::where(var_30, var_41, var_12);
    // return _eval_pt_direct_shifted_3alphas(ctx_Jaref, ctx_jv, efc_D, lo_alpha, hi_alpha, mid_alpha, 0.0)       <L 682>
    _eval_pt_direct_shifted_3alphas_0(var_ctx_Jaref, var_ctx_jv, var_efc_D, var_lo_alpha, var_hi_alpha, var_mid_alpha, var_52, var_53, var_54, var_55);
    ret_0 = var_53;
    ret_1 = var_54;
    ret_2 = var_55;
    return;
}


// /home/cofallen/miniconda3/envs/mjlab/lib/python3.12/site-packages/mujoco_warp/_src/solver.py:203
static CUDA_CALLABLE void _eval_pt_3alphas_0(
    wp::vec_t<3, wp::float32> var_quad,
    wp::float32 var_lo_alpha,
    wp::float32 var_hi_alpha,
    wp::float32 var_mid_alpha,
    wp::vec_t<3, wp::float32> & ret_0,
    wp::vec_t<3, wp::float32> & ret_1,
    wp::vec_t<3, wp::float32> & ret_2)
{
    //---------
    // primal vars
    const wp::int32 var_0 = 0;
    wp::float32 var_1;
    const wp::int32 var_2 = 1;
    wp::float32 var_3;
    const wp::int32 var_4 = 2;
    wp::float32 var_5;
    const wp::float32 var_6 = 2.0;
    wp::float32 var_7;
    wp::float32 var_8;
    wp::float32 var_9;
    wp::float32 var_10;
    wp::float32 var_11;
    wp::float32 var_12;
    wp::float32 var_13;
    wp::float32 var_14;
    const wp::float32 var_15 = 2.0;
    wp::float32 var_16;
    wp::float32 var_17;
    wp::vec_t<3, wp::float32> var_18;
    wp::float32 var_19;
    wp::float32 var_20;
    wp::float32 var_21;
    wp::float32 var_22;
    const wp::float32 var_23 = 2.0;
    wp::float32 var_24;
    wp::float32 var_25;
    wp::vec_t<3, wp::float32> var_26;
    wp::float32 var_27;
    wp::float32 var_28;
    wp::float32 var_29;
    wp::float32 var_30;
    const wp::float32 var_31 = 2.0;
    wp::float32 var_32;
    wp::float32 var_33;
    wp::vec_t<3, wp::float32> var_34;
    //---------
    // forward
    // def _eval_pt_3alphas(quad: wp.vec3, lo_alpha: float, hi_alpha: float, mid_alpha: float) -> tuple[wp.vec3, wp.vec3, wp.vec3]:       <L 204>
    // q0, q1, q2 = quad[0], quad[1], quad[2]                                                 <L 206>
    var_1 = wp::extract(var_quad, var_0);
    var_3 = wp::extract(var_quad, var_2);
    var_5 = wp::extract(var_quad, var_4);
    // hessian = 2.0 * q2                                                                     <L 207>
    var_7 = wp::mul(var_6, var_5);
    // lo_aq2 = lo_alpha * q2                                                                 <L 208>
    var_8 = wp::mul(var_lo_alpha, var_5);
    // hi_aq2 = hi_alpha * q2                                                                 <L 209>
    var_9 = wp::mul(var_hi_alpha, var_5);
    // mid_aq2 = mid_alpha * q2                                                               <L 210>
    var_10 = wp::mul(var_mid_alpha, var_5);
    // return (                                                                               <L 211>
    // wp.vec3(lo_alpha * lo_aq2 + lo_alpha * q1 + q0, 2.0 * lo_aq2 + q1, hessian),           <L 212>
    var_11 = wp::mul(var_lo_alpha, var_8);
    var_12 = wp::mul(var_lo_alpha, var_3);
    var_13 = wp::add(var_11, var_12);
    var_14 = wp::add(var_13, var_1);
    var_16 = wp::mul(var_15, var_8);
    var_17 = wp::add(var_16, var_3);
    var_18 = wp::vec_t<3, wp::float32>(var_14, var_17, var_7);
    // wp.vec3(hi_alpha * hi_aq2 + hi_alpha * q1 + q0, 2.0 * hi_aq2 + q1, hessian),           <L 213>
    var_19 = wp::mul(var_hi_alpha, var_9);
    var_20 = wp::mul(var_hi_alpha, var_3);
    var_21 = wp::add(var_19, var_20);
    var_22 = wp::add(var_21, var_1);
    var_24 = wp::mul(var_23, var_9);
    var_25 = wp::add(var_24, var_3);
    var_26 = wp::vec_t<3, wp::float32>(var_22, var_25, var_7);
    // wp.vec3(mid_alpha * mid_aq2 + mid_alpha * q1 + q0, 2.0 * mid_aq2 + q1, hessian),       <L 214>
    var_27 = wp::mul(var_mid_alpha, var_10);
    var_28 = wp::mul(var_mid_alpha, var_3);
    var_29 = wp::add(var_27, var_28);
    var_30 = wp::add(var_29, var_1);
    var_32 = wp::mul(var_31, var_10);
    var_33 = wp::add(var_32, var_3);
    var_34 = wp::vec_t<3, wp::float32>(var_30, var_33, var_7);
    ret_0 = var_18;
    ret_1 = var_26;
    ret_2 = var_34;
    return;
}


// /home/cofallen/miniconda3/envs/mjlab/lib/python3.12/site-packages/mujoco_warp/_src/solver.py:117
static CUDA_CALLABLE bool _in_bracket_0(
    wp::vec_t<3, wp::float32> var_x,
    wp::vec_t<3, wp::float32> var_y)
{
    //---------
    // primal vars
    bool var_0;
    bool var_1;
    const wp::int32 var_2 = 1;
    wp::float32 var_3;
    const wp::int32 var_4 = 1;
    wp::float32 var_5;
    bool var_6;
    const wp::int32 var_7 = 1;
    wp::float32 var_8;
    const wp::float32 var_9 = 0.0;
    bool var_10;
    bool var_11;
    const wp::int32 var_12 = 1;
    wp::float32 var_13;
    const wp::int32 var_14 = 1;
    wp::float32 var_15;
    bool var_16;
    const wp::int32 var_17 = 1;
    wp::float32 var_18;
    const wp::float32 var_19 = 0.0;
    bool var_20;
    //---------
    // forward
    // def _in_bracket(x: wp.vec3, y: wp.vec3) -> bool:                                       <L 118>
    // return (x[1] < y[1] and y[1] < 0.0) or (x[1] > y[1] and y[1] > 0.0)                    <L 119>
    var_3 = wp::extract(var_x, var_2);
    var_5 = wp::extract(var_y, var_4);
    var_6 = (var_3 < var_5);
    var_1 = var_6;
    if (var_1) {
        var_8 = wp::extract(var_y, var_7);
        var_10 = (var_8 < var_9);
        var_1 = var_1 && var_10;
    }
    var_0 = var_1;
    if (!var_0) {
        var_13 = wp::extract(var_x, var_12);
        var_15 = wp::extract(var_y, var_14);
        var_16 = (var_13 > var_15);
        var_11 = var_16;
        if (var_11) {
            var_18 = wp::extract(var_y, var_17);
            var_20 = (var_18 > var_19);
            var_11 = var_11 && var_20;
        }
        var_0 = var_0 || var_11;
    }
    return var_0;
}


// /home/cofallen/miniconda3/envs/mjlab/lib/python3.12/site-packages/mujoco_warp/_src/solver.py:840
static CUDA_CALLABLE void adj__linesearch_iterative_kernel__locals___syncthreads_0(
    )
{
}


// /home/cofallen/miniconda3/envs/mjlab/lib/python3.12/site-packages/mujoco_warp/_src/solver.py:122
static CUDA_CALLABLE void adj__eval_pt_direct_alpha_zero_0(
    wp::float32 var_jaref,
    wp::float32 var_jv,
    wp::float32 var_d,
    wp::float32 & adj_jaref,
    wp::float32 & adj_jv,
    wp::float32 & adj_d,
    wp::vec_t<3, wp::float32> & adj_ret)
{
	// reverse mode disabled (no backward-enabled kernel depends on this function)
}


// /home/cofallen/miniconda3/envs/mjlab/lib/python3.12/site-packages/mujoco_warp/_src/math.py:0
static CUDA_CALLABLE void adj_safe_div_0(
    wp::float32 var_x,
    wp::float32 var_y,
    wp::float32 & adj_x,
    wp::float32 & adj_y,
    wp::float32 & adj_ret)
{
	// reverse mode disabled (no backward-enabled kernel depends on this function)
}


// /home/cofallen/miniconda3/envs/mjlab/lib/python3.12/site-packages/mujoco_warp/_src/solver.py:223
static CUDA_CALLABLE void adj__eval_frictionloss_pt_0(
    wp::float32 var_x,
    wp::float32 var_f,
    wp::float32 var_rf,
    wp::float32 var_jv,
    wp::float32 var_d,
    wp::float32 & adj_x,
    wp::float32 & adj_f,
    wp::float32 & adj_rf,
    wp::float32 & adj_jv,
    wp::float32 & adj_d,
    wp::vec_t<3, wp::float32> & adj_ret)
{
	// reverse mode disabled (no backward-enabled kernel depends on this function)
}


// /home/cofallen/miniconda3/envs/mjlab/lib/python3.12/site-packages/mujoco_warp/_src/solver.py:562
static CUDA_CALLABLE void adj__compute_efc_eval_pt_alpha_zero_pyramidal_0(
    wp::int32 var_efcid,
    wp::int32 var_ne,
    wp::int32 var_nf,
    wp::float32 var_efc_D,
    wp::array_t<wp::float32> var_efc_frictionloss,
    wp::float32 var_ctx_Jaref,
    wp::float32 var_ctx_jv,
    wp::int32 & adj_efcid,
    wp::int32 & adj_ne,
    wp::int32 & adj_nf,
    wp::float32 & adj_efc_D,
    wp::array_t<wp::float32> & adj_efc_frictionloss,
    wp::float32 & adj_ctx_Jaref,
    wp::float32 & adj_ctx_jv,
    wp::vec_t<3, wp::float32> & adj_ret)
{
	// reverse mode disabled (no backward-enabled kernel depends on this function)
}


// /home/cofallen/miniconda3/envs/mjlab/lib/python3.12/site-packages/mujoco_warp/_src/solver.py:137
static CUDA_CALLABLE void adj__eval_pt_direct_cost_alpha_zero_0(
    wp::float32 var_jaref,
    wp::float32 var_d,
    wp::float32 & adj_jaref,
    wp::float32 & adj_d,
    wp::float32 & adj_ret)
{
	// reverse mode disabled (no backward-enabled kernel depends on this function)
}


// /home/cofallen/miniconda3/envs/mjlab/lib/python3.12/site-packages/mujoco_warp/_src/solver.py:142
static CUDA_CALLABLE void adj__eval_pt_direct_shifted_0(
    wp::float32 var_jaref,
    wp::float32 var_jv,
    wp::float32 var_d,
    wp::float32 var_alpha,
    wp::float32 var_offset,
    wp::float32 & adj_jaref,
    wp::float32 & adj_jv,
    wp::float32 & adj_d,
    wp::float32 & adj_alpha,
    wp::float32 & adj_offset,
    wp::vec_t<3, wp::float32> & adj_ret)
{
	// reverse mode disabled (no backward-enabled kernel depends on this function)
}


// /home/cofallen/miniconda3/envs/mjlab/lib/python3.12/site-packages/mujoco_warp/_src/solver.py:235
static CUDA_CALLABLE void adj__eval_frictionloss_cost_0(
    wp::float32 var_x,
    wp::float32 var_f,
    wp::float32 var_rf,
    wp::float32 var_d,
    wp::float32 & adj_x,
    wp::float32 & adj_f,
    wp::float32 & adj_rf,
    wp::float32 & adj_d,
    wp::float32 & adj_ret)
{
	// reverse mode disabled (no backward-enabled kernel depends on this function)
}


// /home/cofallen/miniconda3/envs/mjlab/lib/python3.12/site-packages/mujoco_warp/_src/solver.py:218
static CUDA_CALLABLE void adj__shift_cost_0(
    wp::vec_t<3, wp::float32> var_pt,
    wp::float32 var_cost0,
    wp::vec_t<3, wp::float32> & adj_pt,
    wp::float32 & adj_cost0,
    wp::vec_t<3, wp::float32> & adj_ret)
{
	// reverse mode disabled (no backward-enabled kernel depends on this function)
}


// /home/cofallen/miniconda3/envs/mjlab/lib/python3.12/site-packages/mujoco_warp/_src/solver.py:466
static CUDA_CALLABLE void adj__compute_efc_eval_pt_pyramidal_0(
    wp::int32 var_efcid,
    wp::float32 var_alpha,
    wp::int32 var_ne,
    wp::int32 var_nf,
    wp::float32 var_efc_D,
    wp::array_t<wp::float32> var_efc_frictionloss,
    wp::float32 var_ctx_Jaref,
    wp::float32 var_ctx_jv,
    wp::int32 & adj_efcid,
    wp::float32 & adj_alpha,
    wp::int32 & adj_ne,
    wp::int32 & adj_nf,
    wp::float32 & adj_efc_D,
    wp::array_t<wp::float32> & adj_efc_frictionloss,
    wp::float32 & adj_ctx_Jaref,
    wp::float32 & adj_ctx_jv,
    wp::vec_t<3, wp::float32> & adj_ret)
{
	// reverse mode disabled (no backward-enabled kernel depends on this function)
}


// /home/cofallen/miniconda3/envs/mjlab/lib/python3.12/site-packages/mujoco_warp/_src/solver.py:192
static CUDA_CALLABLE void adj__eval_pt_0(
    wp::vec_t<3, wp::float32> var_quad,
    wp::float32 var_alpha,
    wp::vec_t<3, wp::float32> & adj_quad,
    wp::float32 & adj_alpha,
    wp::vec_t<3, wp::float32> & adj_ret)
{
	// reverse mode disabled (no backward-enabled kernel depends on this function)
}


// /home/cofallen/miniconda3/envs/mjlab/lib/python3.12/site-packages/mujoco_warp/_src/solver.py:169
static CUDA_CALLABLE void adj__eval_pt_direct_shifted_3alphas_0(
    wp::float32 var_jaref,
    wp::float32 var_jv,
    wp::float32 var_d,
    wp::float32 var_lo_alpha,
    wp::float32 var_hi_alpha,
    wp::float32 var_mid_alpha,
    wp::float32 var_offset,
    wp::vec_t<3, wp::float32> & ret_0,
    wp::vec_t<3, wp::float32> & ret_1,
    wp::vec_t<3, wp::float32> & ret_2,
    wp::float32 & adj_jaref,
    wp::float32 & adj_jv,
    wp::float32 & adj_d,
    wp::float32 & adj_lo_alpha,
    wp::float32 & adj_hi_alpha,
    wp::float32 & adj_mid_alpha,
    wp::float32 & adj_offset,
    wp::vec_t<3, wp::float32> & adj_ret_0,
    wp::vec_t<3, wp::float32> & adj_ret_1,
    wp::vec_t<3, wp::float32> & adj_ret_2)
{
	// reverse mode disabled (no backward-enabled kernel depends on this function)
}


// /home/cofallen/miniconda3/envs/mjlab/lib/python3.12/site-packages/mujoco_warp/_src/solver.py:244
static CUDA_CALLABLE void adj__eval_frictionloss_pt_one_0(
    wp::float32 var_x,
    wp::float32 var_f,
    wp::float32 var_rf,
    wp::float32 var_half_d,
    wp::float32 var_jvD,
    wp::float32 var_hessian,
    wp::float32 var_f_jv,
    wp::float32 & adj_x,
    wp::float32 & adj_f,
    wp::float32 & adj_rf,
    wp::float32 & adj_half_d,
    wp::float32 & adj_jvD,
    wp::float32 & adj_hessian,
    wp::float32 & adj_f_jv,
    wp::vec_t<3, wp::float32> & adj_ret)
{
	// reverse mode disabled (no backward-enabled kernel depends on this function)
}


// /home/cofallen/miniconda3/envs/mjlab/lib/python3.12/site-packages/mujoco_warp/_src/solver.py:255
static CUDA_CALLABLE void adj__eval_frictionloss_pt_3alphas_0(
    wp::float32 var_x_lo,
    wp::float32 var_x_hi,
    wp::float32 var_x_mid,
    wp::float32 var_f,
    wp::float32 var_rf,
    wp::float32 var_jv,
    wp::float32 var_d,
    wp::vec_t<3, wp::float32> & ret_0,
    wp::vec_t<3, wp::float32> & ret_1,
    wp::vec_t<3, wp::float32> & ret_2,
    wp::float32 & adj_x_lo,
    wp::float32 & adj_x_hi,
    wp::float32 & adj_x_mid,
    wp::float32 & adj_f,
    wp::float32 & adj_rf,
    wp::float32 & adj_jv,
    wp::float32 & adj_d,
    wp::vec_t<3, wp::float32> & adj_ret_0,
    wp::vec_t<3, wp::float32> & adj_ret_1,
    wp::vec_t<3, wp::float32> & adj_ret_2)
{
	// reverse mode disabled (no backward-enabled kernel depends on this function)
}


// /home/cofallen/miniconda3/envs/mjlab/lib/python3.12/site-packages/mujoco_warp/_src/solver.py:634
static CUDA_CALLABLE void adj__compute_efc_eval_pt_3alphas_pyramidal_0(
    wp::int32 var_efcid,
    wp::float32 var_lo_alpha,
    wp::float32 var_hi_alpha,
    wp::float32 var_mid_alpha,
    wp::int32 var_ne,
    wp::int32 var_nf,
    wp::float32 var_efc_D,
    wp::array_t<wp::float32> var_efc_frictionloss,
    wp::float32 var_ctx_Jaref,
    wp::float32 var_ctx_jv,
    wp::vec_t<3, wp::float32> & ret_0,
    wp::vec_t<3, wp::float32> & ret_1,
    wp::vec_t<3, wp::float32> & ret_2,
    wp::int32 & adj_efcid,
    wp::float32 & adj_lo_alpha,
    wp::float32 & adj_hi_alpha,
    wp::float32 & adj_mid_alpha,
    wp::int32 & adj_ne,
    wp::int32 & adj_nf,
    wp::float32 & adj_efc_D,
    wp::array_t<wp::float32> & adj_efc_frictionloss,
    wp::float32 & adj_ctx_Jaref,
    wp::float32 & adj_ctx_jv,
    wp::vec_t<3, wp::float32> & adj_ret_0,
    wp::vec_t<3, wp::float32> & adj_ret_1,
    wp::vec_t<3, wp::float32> & adj_ret_2)
{
	// reverse mode disabled (no backward-enabled kernel depends on this function)
}


// /home/cofallen/miniconda3/envs/mjlab/lib/python3.12/site-packages/mujoco_warp/_src/solver.py:203
static CUDA_CALLABLE void adj__eval_pt_3alphas_0(
    wp::vec_t<3, wp::float32> var_quad,
    wp::float32 var_lo_alpha,
    wp::float32 var_hi_alpha,
    wp::float32 var_mid_alpha,
    wp::vec_t<3, wp::float32> & ret_0,
    wp::vec_t<3, wp::float32> & ret_1,
    wp::vec_t<3, wp::float32> & ret_2,
    wp::vec_t<3, wp::float32> & adj_quad,
    wp::float32 & adj_lo_alpha,
    wp::float32 & adj_hi_alpha,
    wp::float32 & adj_mid_alpha,
    wp::vec_t<3, wp::float32> & adj_ret_0,
    wp::vec_t<3, wp::float32> & adj_ret_1,
    wp::vec_t<3, wp::float32> & adj_ret_2)
{
	// reverse mode disabled (no backward-enabled kernel depends on this function)
}


// /home/cofallen/miniconda3/envs/mjlab/lib/python3.12/site-packages/mujoco_warp/_src/solver.py:117
static CUDA_CALLABLE void adj__in_bracket_0(
    wp::vec_t<3, wp::float32> var_x,
    wp::vec_t<3, wp::float32> var_y,
    wp::vec_t<3, wp::float32> & adj_x,
    wp::vec_t<3, wp::float32> & adj_y,
    bool & adj_ret)
{
	// reverse mode disabled (no backward-enabled kernel depends on this function)
}



extern "C" __global__ void _linesearch_iterative_kernel__locals__kernel_cdd7fc06_cuda_kernel_forward(
    wp::launch_bounds_t<2> dim,
    wp::int32 var_nv,
    wp::array_t<wp::float32> var_opt_tolerance,
    wp::array_t<wp::float32> var_opt_ls_tolerance,
    wp::array_t<wp::float32> var_opt_impratio_invsqrt,
    wp::array_t<wp::float32> var_stat_meaninertia,
    wp::array_t<wp::int32> var_ne_in,
    wp::array_t<wp::int32> var_nf_in,
    wp::array_t<wp::int32> var_nefc_in,
    wp::array_t<wp::float32> var_qfrc_smooth_in,
    wp::array_t<wp::vec_t<5, wp::float32>> var_contact_friction_in,
    wp::array_t<wp::int32> var_contact_dim_in,
    wp::array_t<wp::int32> var_contact_efc_address_in,
    wp::array_t<wp::int32> var_efc_type_in,
    wp::array_t<wp::int32> var_efc_id_in,
    wp::array_t<wp::int32> var_efc_J_rownnz_in,
    wp::array_t<wp::int32> var_efc_J_rowadr_in,
    wp::array_t<wp::int32> var_efc_J_colind_in,
    wp::array_t<wp::float32> var_efc_J_in,
    wp::array_t<wp::float32> var_efc_D_in,
    wp::array_t<wp::float32> var_efc_frictionloss_in,
    wp::int32 var_njmax_in,
    wp::array_t<wp::int32> var_nacon_in,
    wp::array_t<bool> var_ctx_search_unchanged_in,
    wp::array_t<wp::float32> var_ctx_Jaref_in,
    wp::array_t<wp::float32> var_ctx_search_in,
    wp::array_t<wp::float32> var_ctx_search_dot_in,
    wp::array_t<wp::float32> var_ctx_mv_in,
    wp::array_t<wp::float32> var_ctx_jv_in,
    wp::array_t<wp::vec_t<3, wp::float32>> var_ctx_quad_in,
    wp::array_t<bool> var_ctx_done_in,
    wp::array_t<wp::float32> var_qacc_out,
    wp::array_t<wp::float32> var_efc_Ma_out,
    wp::array_t<wp::float32> var_ctx_Jaref_out,
    wp::array_t<wp::float32> var_ctx_jv_out,
    wp::array_t<wp::vec_t<3, wp::float32>> var_ctx_quad_out,
    wp::array_t<wp::float32> var_ctx_improvement_out,
    wp::array_t<wp::float32> var_ctx_alpha_out,
    wp::array_t<bool> var_ctx_ls_exhausted_out)
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
        bool* var_2;
        bool var_3;
        bool var_4;
        wp::int32* var_5;
        wp::int32 var_6;
        wp::int32 var_7;
        wp::int32* var_8;
        wp::int32 var_9;
        wp::int32 var_10;
        wp::int32* var_11;
        wp::int32 var_12;
        wp::int32 var_13;
        const bool var_14 = true;
        const bool var_15 = true;
        const bool var_16 = true;
        bool* var_17;
        bool var_18;
        bool var_19;
        wp::int32 var_20;
        wp::range_t var_21;
        wp::int32 var_22;
        const wp::float32 var_23 = 0.0;
        wp::float32 var_24;
        const bool var_25 = false;
        wp::range_t var_26;
        wp::int32 var_27;
        wp::float32* var_28;
        wp::float32* var_29;
        wp::float32 var_30;
        wp::float32 var_31;
        wp::float32 var_32;
        wp::float32 var_33;
        const bool var_34 = false;
        wp::shape_t* var_35;
        const wp::int32 var_36 = 0;
        wp::int32 var_37;
        wp::shape_t var_38;
        wp::int32 var_39;
        wp::float32* var_40;
        wp::float32 var_41;
        wp::float32 var_42;
        wp::shape_t* var_43;
        const wp::int32 var_44 = 0;
        wp::int32 var_45;
        wp::shape_t var_46;
        wp::int32 var_47;
        wp::float32* var_48;
        wp::float32 var_49;
        wp::float32 var_50;
        wp::float32* var_51;
        wp::float32 var_52;
        wp::float32 var_53;
        wp::shape_t* var_54;
        const wp::int32 var_55 = 0;
        wp::int32 var_56;
        wp::shape_t var_57;
        wp::int32 var_58;
        wp::float32* var_59;
        wp::float32 var_60;
        wp::float32 var_61;
        wp::float32 var_62;
        wp::float32 var_63;
        wp::float32 var_64;
        wp::float32 var_65;
        wp::float32 var_66;
        const wp::float32 var_67 = 1e-06;
        wp::float32 var_68;
        const wp::float32 var_69 = 0.0;
        wp::vec_t<3, wp::float32> var_70;
        wp::int32 var_71;
        wp::range_t var_72;
        wp::int32 var_73;
        const bool var_74 = false;
        wp::float32* var_75;
        wp::array_t<wp::float32> var_76;
        wp::float32* var_77;
        wp::float32* var_78;
        wp::vec_t<3, wp::float32> var_79;
        wp::float32 var_80;
        wp::float32 var_81;
        wp::float32 var_82;
        wp::vec_t<3, wp::float32> var_83;
        const bool var_84 = true;
        wp::tile_register_t<wp::vec_t<3, wp::float32>,wp::tile_layout_register_t<wp::tile_shape_t<32>>> var_85 = wp::tile_register_t<wp::vec_t<3, wp::float32>,wp::tile_layout_register_t<wp::tile_shape_t<32>>>{};
        wp::tile_shared_t<wp::vec_t<3, wp::float32>,wp::tile_layout_strided_t<wp::tile_shape_t<1>, wp::tile_stride_t<1>>, true> var_86 = wp::tile_alloc_empty<wp::vec_t<3, wp::float32>,wp::tile_shape_t<1>,wp::tile_stride_t<1>,false>();
        const wp::float32 var_87 = 0.0;
        wp::vec_t<2, wp::float32> var_88;
        wp::int32 var_89;
        wp::range_t var_90;
        wp::int32 var_91;
        wp::float32* var_92;
        wp::float32 var_93;
        wp::float32 var_94;
        wp::float32* var_95;
        wp::float32* var_96;
        wp::float32 var_97;
        wp::float32 var_98;
        wp::float32 var_99;
        wp::float32 var_100;
        const wp::float32 var_101 = 0.5;
        wp::float32 var_102;
        wp::float32* var_103;
        wp::float32 var_104;
        wp::float32 var_105;
        wp::vec_t<2, wp::float32> var_106;
        wp::vec_t<2, wp::float32> var_107;
        const bool var_108 = true;
        wp::tile_register_t<wp::vec_t<2, wp::float32>,wp::tile_layout_register_t<wp::tile_shape_t<32>>> var_109 = wp::tile_register_t<wp::vec_t<2, wp::float32>,wp::tile_layout_register_t<wp::tile_shape_t<32>>>{};
        wp::tile_shared_t<wp::vec_t<2, wp::float32>,wp::tile_layout_strided_t<wp::tile_shape_t<1>, wp::tile_stride_t<1>>, true> var_110 = wp::tile_alloc_empty<wp::vec_t<2, wp::float32>,wp::tile_shape_t<1>,wp::tile_stride_t<1>,false>();
        const wp::int32 var_111 = 0;
        wp::vec_t<2, wp::float32> var_112;
        const wp::float32 var_113 = 0.0;
        const wp::int32 var_114 = 0;
        wp::float32 var_115;
        const wp::int32 var_116 = 1;
        wp::float32 var_117;
        wp::vec_t<3, wp::float32> var_118;
        const wp::int32 var_119 = 0;
        wp::float32 var_120;
        const wp::int32 var_121 = 1;
        wp::float32 var_122;
        const wp::float32 var_123 = 2.0;
        const wp::int32 var_124 = 2;
        wp::float32 var_125;
        wp::float32 var_126;
        wp::vec_t<3, wp::float32> var_127;
        const wp::int32 var_128 = 0;
        wp::vec_t<3, wp::float32> var_129;
        wp::vec_t<3, wp::float32> var_130;
        const wp::float32 var_131 = 0.0;
        const wp::int32 var_132 = 1;
        wp::float32 var_133;
        const wp::int32 var_134 = 2;
        wp::float32 var_135;
        wp::vec_t<3, wp::float32> var_136;
        const wp::float32 var_137 = 0.0;
        wp::float32 var_138;
        const bool var_139 = true;
        const wp::int32 var_140 = 0;
        wp::vec_t<3, wp::float32> var_141;
        const wp::float32 var_142 = 2.0;
        const wp::int32 var_143 = 0;
        wp::float32 var_144;
        const wp::float32 var_145 = 0.0;
        wp::float32 var_146;
        wp::float32 var_147;
        const wp::int32 var_148 = 2;
        wp::float32 var_149;
        const wp::float32 var_150 = 0.0;
        wp::float32 var_151;
        wp::float32 var_152;
        wp::float32 var_153;
        const wp::int32 var_154 = 1;
        wp::float32 var_155;
        wp::float32 var_156;
        wp::float32 var_157;
        const wp::float32 var_158 = 9.5367432e-07;
        const wp::float32 var_159 = 1.0;
        const wp::int32 var_160 = 2;
        wp::float32 var_161;
        wp::float32 var_162;
        wp::float32 var_163;
        wp::float32 var_164;
        const wp::int32 var_165 = 1;
        wp::float32 var_166;
        const wp::int32 var_167 = 2;
        wp::float32 var_168;
        wp::float32 var_169;
        wp::float32 var_170;
        const wp::float32 var_171 = 0.0;
        wp::vec_t<3, wp::float32> var_172;
        wp::int32 var_173;
        wp::range_t var_174;
        wp::int32 var_175;
        const bool var_176 = false;
        wp::float32* var_177;
        wp::array_t<wp::float32> var_178;
        wp::float32* var_179;
        wp::float32* var_180;
        wp::vec_t<3, wp::float32> var_181;
        wp::float32 var_182;
        wp::float32 var_183;
        wp::float32 var_184;
        wp::vec_t<3, wp::float32> var_185;
        const bool var_186 = true;
        wp::tile_register_t<wp::vec_t<3, wp::float32>,wp::tile_layout_register_t<wp::tile_shape_t<32>>> var_187 = wp::tile_register_t<wp::vec_t<3, wp::float32>,wp::tile_layout_register_t<wp::tile_shape_t<32>>>{};
        wp::tile_shared_t<wp::vec_t<3, wp::float32>,wp::tile_layout_strided_t<wp::tile_shape_t<1>, wp::tile_stride_t<1>>, true> var_188 = wp::tile_alloc_empty<wp::vec_t<3, wp::float32>,wp::tile_shape_t<1>,wp::tile_stride_t<1>,false>();
        wp::vec_t<3, wp::float32> var_189;
        const wp::int32 var_190 = 0;
        wp::vec_t<3, wp::float32> var_191;
        wp::vec_t<3, wp::float32> var_192;
        bool var_193;
        const wp::int32 var_194 = 1;
        wp::float32 var_195;
        wp::float32 var_196;
        bool var_197;
        const wp::int32 var_198 = 0;
        wp::float32 var_199;
        const wp::float32 var_200 = 0.0;
        bool var_201;
        bool var_202;
        const wp::float32 var_203 = 0.0;
        wp::float32 var_204;
        const wp::float32 var_205 = 0.0;
        wp::float32 var_206;
        const wp::int32 var_207 = 1;
        wp::float32 var_208;
        const wp::int32 var_209 = 1;
        wp::float32 var_210;
        bool var_211;
        wp::vec_t<3, wp::float32> var_212;
        const wp::float32 var_213 = 0.0;
        wp::float32 var_214;
        wp::vec_t<3, wp::float32> var_215;
        const wp::float32 var_216 = 0.0;
        wp::float32 var_217;
        const wp::int32 var_218 = 50;
        wp::range_t var_219;
        wp::int32 var_220;
        const wp::int32 var_221 = 1;
        wp::float32 var_222;
        const wp::int32 var_223 = 2;
        wp::float32 var_224;
        wp::float32 var_225;
        wp::float32 var_226;
        const wp::int32 var_227 = 1;
        wp::float32 var_228;
        const wp::int32 var_229 = 2;
        wp::float32 var_230;
        wp::float32 var_231;
        wp::float32 var_232;
        const wp::float32 var_233 = 0.5;
        wp::float32 var_234;
        wp::float32 var_235;
        const wp::float32 var_236 = 0.0;
        wp::vec_t<3, wp::float32> var_237;
        const wp::float32 var_238 = 0.0;
        wp::vec_t<3, wp::float32> var_239;
        const wp::float32 var_240 = 0.0;
        wp::vec_t<3, wp::float32> var_241;
        wp::int32 var_242;
        wp::range_t var_243;
        wp::int32 var_244;
        const bool var_245 = false;
        wp::float32* var_246;
        wp::array_t<wp::float32> var_247;
        wp::float32* var_248;
        wp::float32* var_249;
        wp::vec_t<3, wp::float32> var_250;
        wp::vec_t<3, wp::float32> var_251;
        wp::vec_t<3, wp::float32> var_252;
        wp::float32 var_253;
        wp::float32 var_254;
        wp::float32 var_255;
        wp::vec_t<3, wp::float32> var_256;
        wp::vec_t<3, wp::float32> var_257;
        wp::vec_t<3, wp::float32> var_258;
        const wp::int32 var_259 = 0;
        wp::float32 var_260;
        const wp::int32 var_261 = 0;
        wp::float32 var_262;
        const wp::int32 var_263 = 0;
        wp::float32 var_264;
        const wp::int32 var_265 = 1;
        wp::float32 var_266;
        const wp::int32 var_267 = 1;
        wp::float32 var_268;
        const wp::int32 var_269 = 1;
        wp::float32 var_270;
        const wp::int32 var_271 = 2;
        wp::float32 var_272;
        const wp::int32 var_273 = 2;
        wp::float32 var_274;
        const wp::int32 var_275 = 2;
        wp::float32 var_276;
        wp::mat_t<3, 3, wp::float32> var_277;
        const bool var_278 = true;
        wp::tile_register_t<wp::mat_t<3, 3, wp::float32>,wp::tile_layout_register_t<wp::tile_shape_t<32>>> var_279 = wp::tile_register_t<wp::mat_t<3, 3, wp::float32>,wp::tile_layout_register_t<wp::tile_shape_t<32>>>{};
        wp::tile_shared_t<wp::mat_t<3, 3, wp::float32>,wp::tile_layout_strided_t<wp::tile_shape_t<1>, wp::tile_stride_t<1>>, true> var_280 = wp::tile_alloc_empty<wp::mat_t<3, 3, wp::float32>,wp::tile_shape_t<1>,wp::tile_stride_t<1>,false>();
        const wp::int32 var_281 = 0;
        wp::mat_t<3, 3, wp::float32> var_282;
        wp::vec_t<3, wp::float32> var_283;
        wp::vec_t<3, wp::float32> var_284;
        wp::vec_t<3, wp::float32> var_285;
        const wp::int32 var_286 = 0;
        const wp::int32 var_287 = 0;
        wp::float32 var_288;
        const wp::int32 var_289 = 1;
        const wp::int32 var_290 = 0;
        wp::float32 var_291;
        const wp::int32 var_292 = 2;
        const wp::int32 var_293 = 0;
        wp::float32 var_294;
        wp::vec_t<3, wp::float32> var_295;
        wp::vec_t<3, wp::float32> var_296;
        const wp::int32 var_297 = 0;
        const wp::int32 var_298 = 1;
        wp::float32 var_299;
        const wp::int32 var_300 = 1;
        const wp::int32 var_301 = 1;
        wp::float32 var_302;
        const wp::int32 var_303 = 2;
        const wp::int32 var_304 = 1;
        wp::float32 var_305;
        wp::vec_t<3, wp::float32> var_306;
        wp::vec_t<3, wp::float32> var_307;
        const wp::int32 var_308 = 0;
        const wp::int32 var_309 = 2;
        wp::float32 var_310;
        const wp::int32 var_311 = 1;
        const wp::int32 var_312 = 2;
        wp::float32 var_313;
        const wp::int32 var_314 = 2;
        const wp::int32 var_315 = 2;
        wp::float32 var_316;
        wp::vec_t<3, wp::float32> var_317;
        wp::vec_t<3, wp::float32> var_318;
        bool var_319;
        wp::vec_t<3, wp::float32> var_320;
        wp::float32 var_321;
        bool var_322;
        wp::vec_t<3, wp::float32> var_323;
        wp::float32 var_324;
        bool var_325;
        wp::vec_t<3, wp::float32> var_326;
        wp::float32 var_327;
        bool var_328;
        bool var_329;
        wp::vec_t<3, wp::float32> var_330;
        wp::float32 var_331;
        bool var_332;
        wp::vec_t<3, wp::float32> var_333;
        wp::float32 var_334;
        bool var_335;
        wp::vec_t<3, wp::float32> var_336;
        wp::float32 var_337;
        bool var_338;
        bool var_339;
        bool var_340;
        bool var_341;
        bool var_342;
        bool var_343;
        const wp::int32 var_344 = 0;
        wp::float32 var_345;
        const wp::float32 var_346 = 0.0;
        bool var_347;
        const wp::int32 var_348 = 1;
        wp::float32 var_349;
        const wp::float32 var_350 = 0.0;
        bool var_351;
        const wp::int32 var_352 = 1;
        wp::float32 var_353;
        wp::float32 var_354;
        bool var_355;
        bool var_356;
        const wp::int32 var_357 = 0;
        wp::float32 var_358;
        const wp::float32 var_359 = 0.0;
        bool var_360;
        const wp::int32 var_361 = 1;
        wp::float32 var_362;
        const wp::float32 var_363 = 0.0;
        bool var_364;
        const wp::int32 var_365 = 1;
        wp::float32 var_366;
        bool var_367;
        bool var_368;
        const wp::int32 var_369 = 0;
        wp::float32 var_370;
        const wp::float32 var_371 = 0.0;
        bool var_372;
        const wp::int32 var_373 = 0;
        wp::float32 var_374;
        const wp::float32 var_375 = 0.0;
        bool var_376;
        const wp::int32 var_377 = 0;
        wp::float32 var_378;
        const wp::int32 var_379 = 0;
        wp::float32 var_380;
        bool var_381;
        wp::float32 var_382;
        const wp::int32 var_383 = 0;
        wp::float32 var_384;
        const wp::int32 var_385 = 0;
        wp::float32 var_386;
        wp::float32 var_387;
        wp::float32 var_388;
        wp::float32 var_389;
        wp::float32 var_390;
        wp::int32 var_391;
        wp::float32 var_392;
        wp::float32 var_393;
        wp::vec_t<3, wp::float32> var_394;
        wp::float32 var_395;
        wp::vec_t<3, wp::float32> var_396;
        wp::float32 var_397;
        wp::float32 var_398;
        const wp::int32 var_399 = 0;
        wp::float32 var_400;
        wp::float32 var_401;
        wp::float32 var_402;
        wp::float32 var_403;
        wp::int32 var_404;
        wp::range_t var_405;
        wp::int32 var_406;
        wp::float32* var_407;
        wp::float32 var_408;
        wp::float32 var_409;
        wp::float32 var_410;
        wp::float32* var_411;
        wp::float32 var_412;
        wp::float32 var_413;
        wp::float32 var_414;
        wp::int32 var_415;
        wp::range_t var_416;
        wp::int32 var_417;
        wp::float32* var_418;
        wp::float32 var_419;
        wp::float32 var_420;
        wp::float32 var_421;
        const wp::int32 var_422 = 0;
        bool var_423;
        const bool var_424 = true;
        wp::float32 var_425;
        bool var_426;
        //---------
        // forward
        // def kernel(                                                                            <L 855>
        // worldid, tid = wp.tid()                                                                <L 900>
        builtin_tid2d(var_0, var_1);
        // if ctx_done_in[worldid]:                                                               <L 902>
        var_2 = wp::address(var_ctx_done_in, var_0);
        var_3 = wp::load(var_2);
        if (var_3) {
            // return                                                                             <L 903>
            continue;
        }
        var_4 = wp::load(var_2);
        // ne = ne_in[worldid]                                                                    <L 905>
        var_5 = wp::address(var_ne_in, var_0);
        var_7 = wp::load(var_5);
        var_6 = wp::copy(var_7);
        // nf = nf_in[worldid]                                                                    <L 906>
        var_8 = wp::address(var_nf_in, var_0);
        var_10 = wp::load(var_8);
        var_9 = wp::copy(var_10);
        // nefc = wp.min(njmax_in, nefc_in[worldid])                                              <L 907>
        var_11 = wp::address(var_nefc_in, var_0);
        var_13 = wp::load(var_11);
        var_12 = wp::min(var_njmax_in, var_13);
        // if wp.static(FUSE_JV):                                                                 <L 911>
        // recompute_jv = True                                                                    <L 912>
        // if wp.static(INCREMENTAL):                                                             <L 913>
        // recompute_jv = not ctx_search_unchanged_in[worldid]                                    <L 914>
        var_17 = wp::address(var_ctx_search_unchanged_in, var_0);
        var_19 = wp::load(var_17);
        var_18 = wp::unot(var_19);
        // if recompute_jv:                                                                       <L 915>
        if (var_18) {
            // for efcid in range(tid, nefc, wp.block_dim()):                                     <L 916>
            var_20 = builtin_block_dim();
            var_21 = wp::range(var_1, var_12, var_20);
            start_for_1:;
                if (iter_cmp(var_21) == 0) goto end_for_1;
                var_22 = wp::iter_next(var_21);
                // jv = float(0.0)                                                                <L 917>
                var_24 = wp::float(var_23);
                // if wp.static(IS_SPARSE):                                                       <L 918>
                // for i in range(nv):                                                            <L 926>
                var_26 = wp::range(var_nv);
                start_for_3:;
                    if (iter_cmp(var_26) == 0) goto end_for_3;
                    var_27 = wp::iter_next(var_26);
                    // jv += efc_J_in[worldid, efcid, i] * ctx_search_in[worldid, i]              <L 927>
                    var_28 = wp::address(var_efc_J_in, var_0, var_22, var_27);
                    var_29 = wp::address(var_ctx_search_in, var_0, var_27);
                    var_31 = wp::load(var_28);
                    var_32 = wp::load(var_29);
                    var_30 = wp::mul(var_31, var_32);
                    var_33 = wp::add(var_24, var_30);
                    wp::assign(var_24, var_33);
                    goto start_for_3;
                end_for_3:;
                // ctx_jv_out[worldid, efcid] = jv                                                <L 928>
                wp::array_store(var_ctx_jv_out, var_0, var_22, var_24);
                goto start_for_1;
            end_for_1:;
        }
        // _syncthreads()  # ensure all jv values are written before reading                      <L 930>
        _linesearch_iterative_kernel__locals___syncthreads_0();
        // if wp.static(IS_ELLIPTIC):                                                             <L 935>
        // tolerance = opt_tolerance[worldid % opt_tolerance.shape[0]]                            <L 996>
        var_35 = &(var_opt_tolerance.shape);
        var_38 = wp::load(var_35);
        var_37 = wp::extract(var_38, var_36);
        var_39 = wp::mod(var_0, var_37);
        var_40 = wp::address(var_opt_tolerance, var_39);
        var_42 = wp::load(var_40);
        var_41 = wp::copy(var_42);
        // ls_tolerance = opt_ls_tolerance[worldid % opt_ls_tolerance.shape[0]]                   <L 997>
        var_43 = &(var_opt_ls_tolerance.shape);
        var_46 = wp::load(var_43);
        var_45 = wp::extract(var_46, var_44);
        var_47 = wp::mod(var_0, var_45);
        var_48 = wp::address(var_opt_ls_tolerance, var_47);
        var_50 = wp::load(var_48);
        var_49 = wp::copy(var_50);
        // snorm = wp.sqrt(ctx_search_dot_in[worldid])                                            <L 998>
        var_51 = wp::address(var_ctx_search_dot_in, var_0);
        var_53 = wp::load(var_51);
        var_52 = wp::sqrt(var_53);
        // meaninertia = stat_meaninertia[worldid % stat_meaninertia.shape[0]]                    <L 999>
        var_54 = &(var_stat_meaninertia.shape);
        var_57 = wp::load(var_54);
        var_56 = wp::extract(var_57, var_55);
        var_58 = wp::mod(var_0, var_56);
        var_59 = wp::address(var_stat_meaninertia, var_58);
        var_61 = wp::load(var_59);
        var_60 = wp::copy(var_61);
        // scale = meaninertia * wp.float(nv)                                                     <L 1000>
        var_62 = wp::float(var_nv);
        var_63 = wp::mul(var_60, var_62);
        // gtol = wp.max(tolerance * ls_tolerance * snorm * scale, 1e-6)                          <L 1001>
        var_64 = wp::mul(var_41, var_49);
        var_65 = wp::mul(var_64, var_52);
        var_66 = wp::mul(var_65, var_63);
        var_68 = wp::max(var_66, var_67);
        // local_p0 = wp.vec3(0.0)                                                                <L 1004>
        var_70 = wp::vec_t<3, wp::float32>(var_69);
        // for efcid in range(tid, nefc, wp.block_dim()):                                         <L 1005>
        var_71 = builtin_block_dim();
        var_72 = wp::range(var_1, var_12, var_71);
        start_for_5:;
            if (iter_cmp(var_72) == 0) goto end_for_5;
            var_73 = wp::iter_next(var_72);
            // if wp.static(IS_ELLIPTIC):                                                         <L 1006>
            // local_p0 += _compute_efc_eval_pt_alpha_zero(                                       <L 1043>
            // efcid,                                                                             <L 1044>
            // ne,                                                                                <L 1045>
            // nf,                                                                                <L 1046>
            // efc_D_in[worldid, efcid],                                                          <L 1047>
            var_75 = wp::address(var_efc_D_in, var_0, var_73);
            // efc_frictionloss_in[worldid],                                                      <L 1048>
            var_76 = wp::view(var_efc_frictionloss_in, var_0);
            // ctx_Jaref_in[worldid, efcid],                                                      <L 1049>
            var_77 = wp::address(var_ctx_Jaref_in, var_0, var_73);
            // ctx_jv_in[worldid, efcid],                                                         <L 1050>
            var_78 = wp::address(var_ctx_jv_in, var_0, var_73);
            var_80 = wp::load(var_75);
            var_81 = wp::load(var_77);
            var_82 = wp::load(var_78);
            var_79 = _compute_efc_eval_pt_alpha_zero_pyramidal_0(var_73, var_6, var_9, var_80, var_76, var_81, var_82);
            // local_p0 += _compute_efc_eval_pt_alpha_zero(                                       <L 1043>
            var_83 = wp::add(var_70, var_79);
            wp::assign(var_70, var_83);
            goto start_for_5;
        end_for_5:;
        // p0_tile = wp.tile(local_p0, preserve_type=True)                                        <L 1056>
        var_85 = wp::tile<wp::vec_t<3, wp::float32>>(var_70);
        // p0_sum = wp.tile_reduce(wp.add, p0_tile)                                               <L 1057>
        var_86 = wp::tile_reduce(wp::add, var_85);
        // local_gauss = wp.vec2(0.0)                                                             <L 1060>
        var_88 = wp::vec_t<2, wp::float32>(var_87);
        // for dofid in range(tid, nv, wp.block_dim()):                                           <L 1061>
        var_89 = builtin_block_dim();
        var_90 = wp::range(var_1, var_nv, var_89);
        start_for_7:;
            if (iter_cmp(var_90) == 0) goto end_for_7;
            var_91 = wp::iter_next(var_90);
            // search = ctx_search_in[worldid, dofid]                                             <L 1062>
            var_92 = wp::address(var_ctx_search_in, var_0, var_91);
            var_94 = wp::load(var_92);
            var_93 = wp::copy(var_94);
            // local_gauss += wp.vec2(                                                            <L 1063>
            // search * (efc_Ma_out[worldid, dofid] - qfrc_smooth_in[worldid, dofid]),            <L 1064>
            var_95 = wp::address(var_efc_Ma_out, var_0, var_91);
            var_96 = wp::address(var_qfrc_smooth_in, var_0, var_91);
            var_98 = wp::load(var_95);
            var_99 = wp::load(var_96);
            var_97 = wp::sub(var_98, var_99);
            var_100 = wp::mul(var_93, var_97);
            // 0.5 * search * ctx_mv_in[worldid, dofid],                                          <L 1065>
            var_102 = wp::mul(var_101, var_93);
            var_103 = wp::address(var_ctx_mv_in, var_0, var_91);
            var_105 = wp::load(var_103);
            var_104 = wp::mul(var_102, var_105);
            var_106 = wp::vec_t<2, wp::float32>(var_100, var_104);
            // local_gauss += wp.vec2(                                                            <L 1063>
            var_107 = wp::add(var_88, var_106);
            wp::assign(var_88, var_107);
            goto start_for_7;
        end_for_7:;
        // gauss_tile = wp.tile(local_gauss, preserve_type=True)                                  <L 1068>
        var_109 = wp::tile<wp::vec_t<2, wp::float32>>(var_88);
        // gauss_sum = wp.tile_reduce(wp.add, gauss_tile)                                         <L 1069>
        var_110 = wp::tile_reduce(wp::add, var_109);
        // gauss_reduced = gauss_sum[0]                                                           <L 1070>
        var_112 = wp::tile_extract(var_110, var_111);
        // ctx_quad_gauss = wp.vec3(0.0, gauss_reduced[0], gauss_reduced[1])                      <L 1071>
        var_115 = wp::extract(var_112, var_114);
        var_117 = wp::extract(var_112, var_116);
        var_118 = wp::vec_t<3, wp::float32>(var_113, var_115, var_117);
        // p0 = wp.vec3(ctx_quad_gauss[0], ctx_quad_gauss[1], 2.0 * ctx_quad_gauss[2]) + p0_sum[0]       <L 1074>
        var_120 = wp::extract(var_118, var_119);
        var_122 = wp::extract(var_118, var_121);
        var_125 = wp::extract(var_118, var_124);
        var_126 = wp::mul(var_123, var_125);
        var_127 = wp::vec_t<3, wp::float32>(var_120, var_122, var_126);
        var_129 = wp::tile_extract(var_86, var_128);
        var_130 = wp::add(var_127, var_129);
        // p0_delta = wp.vec3(0.0, p0[1], p0[2])                                                  <L 1075>
        var_133 = wp::extract(var_130, var_132);
        var_135 = wp::extract(var_130, var_134);
        var_136 = wp::vec_t<3, wp::float32>(var_131, var_133, var_135);
        // noise_floor = float(0.0)                                                               <L 1085>
        var_138 = wp::float(var_137);
        // if wp.static(INCREMENTAL):                                                             <L 1086>
        // rows = p0_sum[0]                                                                       <L 1087>
        var_141 = wp::tile_extract(var_86, var_140);
        // q1_abs = wp.sqrt(2.0 * wp.max(rows[0], 0.0) * wp.max(rows[2], 0.0)) + wp.abs(ctx_quad_gauss[1])       <L 1088>
        var_144 = wp::extract(var_141, var_143);
        var_146 = wp::max(var_144, var_145);
        var_147 = wp::mul(var_142, var_146);
        var_149 = wp::extract(var_141, var_148);
        var_151 = wp::max(var_149, var_150);
        var_152 = wp::mul(var_147, var_151);
        var_153 = wp::sqrt(var_152);
        var_155 = wp::extract(var_118, var_154);
        var_156 = wp::abs(var_155);
        var_157 = wp::add(var_153, var_156);
        // noise_floor = _ALPHA_NOISE_EPS * wp.max(1.0, math.safe_div(q1_abs, p0[2]))             <L 1089>
        var_161 = wp::extract(var_130, var_160);
        var_162 = safe_div_0(var_157, var_161);
        var_163 = wp::max(var_159, var_162);
        var_164 = wp::mul(var_158, var_163);
        // lo_alpha_in = -math.safe_div(p0[1], p0[2])                                             <L 1092>
        var_166 = wp::extract(var_130, var_165);
        var_168 = wp::extract(var_130, var_167);
        var_169 = safe_div_0(var_166, var_168);
        var_170 = wp::neg(var_169);
        // local_lo_in = wp.vec3(0.0)                                                             <L 1094>
        var_172 = wp::vec_t<3, wp::float32>(var_171);
        // for efcid in range(tid, nefc, wp.block_dim()):                                         <L 1095>
        var_173 = builtin_block_dim();
        var_174 = wp::range(var_1, var_12, var_173);
        start_for_9:;
            if (iter_cmp(var_174) == 0) goto end_for_9;
            var_175 = wp::iter_next(var_174);
            // if wp.static(IS_ELLIPTIC):                                                         <L 1096>
            // local_lo_in += _compute_efc_eval_pt(                                               <L 1134>
            // efcid,                                                                             <L 1135>
            // lo_alpha_in,                                                                       <L 1136>
            // ne,                                                                                <L 1137>
            // nf,                                                                                <L 1138>
            // efc_D_in[worldid, efcid],                                                          <L 1139>
            var_177 = wp::address(var_efc_D_in, var_0, var_175);
            // efc_frictionloss_in[worldid],                                                      <L 1140>
            var_178 = wp::view(var_efc_frictionloss_in, var_0);
            // ctx_Jaref_in[worldid, efcid],                                                      <L 1141>
            var_179 = wp::address(var_ctx_Jaref_in, var_0, var_175);
            // ctx_jv_in[worldid, efcid],                                                         <L 1142>
            var_180 = wp::address(var_ctx_jv_in, var_0, var_175);
            var_182 = wp::load(var_177);
            var_183 = wp::load(var_179);
            var_184 = wp::load(var_180);
            var_181 = _compute_efc_eval_pt_pyramidal_0(var_175, var_170, var_6, var_9, var_182, var_178, var_183, var_184);
            // local_lo_in += _compute_efc_eval_pt(                                               <L 1134>
            var_185 = wp::add(var_172, var_181);
            wp::assign(var_172, var_185);
            goto start_for_9;
        end_for_9:;
        // lo_in_tile = wp.tile(local_lo_in, preserve_type=True)                                  <L 1145>
        var_187 = wp::tile<wp::vec_t<3, wp::float32>>(var_172);
        // lo_in_sum = wp.tile_reduce(wp.add, lo_in_tile)                                         <L 1146>
        var_188 = wp::tile_reduce(wp::add, var_187);
        // lo_in = _eval_pt(ctx_quad_gauss, lo_alpha_in) + lo_in_sum[0]                           <L 1147>
        var_189 = _eval_pt_0(var_118, var_170);
        var_191 = wp::tile_extract(var_188, var_190);
        var_192 = wp::add(var_189, var_191);
        // initial_converged = wp.abs(lo_in[1]) < gtol and lo_in[0] < 0.0                         <L 1150>
        var_195 = wp::extract(var_192, var_194);
        var_196 = wp::abs(var_195);
        var_197 = (var_196 < var_68);
        var_193 = var_197;
        if (var_193) {
            var_199 = wp::extract(var_192, var_198);
            var_201 = (var_199 < var_200);
            var_193 = var_193 && var_201;
        }
        // if not initial_converged:                                                              <L 1153>
        var_202 = wp::unot(var_193);
        if (var_202) {
            // alpha = float(0.0)                                                                 <L 1154>
            var_204 = wp::float(var_203);
            // improvement = float(0.0)                                                           <L 1155>
            var_206 = wp::float(var_205);
            // lo_less = lo_in[1] < p0[1]                                                         <L 1158>
            var_208 = wp::extract(var_192, var_207);
            var_210 = wp::extract(var_130, var_209);
            var_211 = (var_208 < var_210);
            // lo = wp.where(lo_less, lo_in, p0_delta)                                            <L 1159>
            var_212 = wp::where(var_211, var_192, var_136);
            // lo_alpha = wp.where(lo_less, lo_alpha_in, 0.0)                                     <L 1160>
            var_214 = wp::where(var_211, var_170, var_213);
            // hi = wp.where(lo_less, p0_delta, lo_in)                                            <L 1161>
            var_215 = wp::where(var_211, var_136, var_192);
            // hi_alpha = wp.where(lo_less, 0.0, lo_alpha_in)                                     <L 1162>
            var_217 = wp::where(var_211, var_216, var_170);
            // for _ in range(LS_ITERATIONS):                                                     <L 1164>
            var_219 = wp::range(var_218);
            start_for_11:;
                if (iter_cmp(var_219) == 0) goto end_for_11;
                var_220 = wp::iter_next(var_219);
                // lo_next_alpha = lo_alpha - math.safe_div(lo[1], lo[2])                         <L 1165>
                var_222 = wp::extract(var_212, var_221);
                var_224 = wp::extract(var_212, var_223);
                var_225 = safe_div_0(var_222, var_224);
                var_226 = wp::sub(var_214, var_225);
                // hi_next_alpha = hi_alpha - math.safe_div(hi[1], hi[2])                         <L 1166>
                var_228 = wp::extract(var_215, var_227);
                var_230 = wp::extract(var_215, var_229);
                var_231 = safe_div_0(var_228, var_230);
                var_232 = wp::sub(var_217, var_231);
                // mid_alpha = 0.5 * (lo_alpha + hi_alpha)                                        <L 1167>
                var_234 = wp::add(var_214, var_217);
                var_235 = wp::mul(var_233, var_234);
                // local_lo = wp.vec3(0.0)                                                        <L 1169>
                var_237 = wp::vec_t<3, wp::float32>(var_236);
                // local_hi = wp.vec3(0.0)                                                        <L 1170>
                var_239 = wp::vec_t<3, wp::float32>(var_238);
                // local_mid = wp.vec3(0.0)                                                       <L 1171>
                var_241 = wp::vec_t<3, wp::float32>(var_240);
                // for efcid in range(tid, nefc, wp.block_dim()):                                 <L 1173>
                var_242 = builtin_block_dim();
                var_243 = wp::range(var_1, var_12, var_242);
                start_for_13:;
                    if (iter_cmp(var_243) == 0) goto end_for_13;
                    var_244 = wp::iter_next(var_243);
                    // if wp.static(IS_ELLIPTIC):                                                 <L 1174>
                    // r_lo, r_hi, r_mid = _compute_efc_eval_pt_3alphas(                          <L 1214>
                    // efcid,                                                                     <L 1215>
                    // lo_next_alpha,                                                             <L 1216>
                    // hi_next_alpha,                                                             <L 1217>
                    // mid_alpha,                                                                 <L 1218>
                    // ne,                                                                        <L 1219>
                    // nf,                                                                        <L 1220>
                    // efc_D_in[worldid, efcid],                                                  <L 1221>
                    var_246 = wp::address(var_efc_D_in, var_0, var_244);
                    // efc_frictionloss_in[worldid],                                              <L 1222>
                    var_247 = wp::view(var_efc_frictionloss_in, var_0);
                    // ctx_Jaref_in[worldid, efcid],                                              <L 1223>
                    var_248 = wp::address(var_ctx_Jaref_in, var_0, var_244);
                    // ctx_jv_in[worldid, efcid],                                                 <L 1224>
                    var_249 = wp::address(var_ctx_jv_in, var_0, var_244);
                    var_253 = wp::load(var_246);
                    var_254 = wp::load(var_248);
                    var_255 = wp::load(var_249);
                    _compute_efc_eval_pt_3alphas_pyramidal_0(var_244, var_226, var_232, var_235, var_6, var_9, var_253, var_247, var_254, var_255, var_250, var_251, var_252);
                    // local_lo += r_lo                                                           <L 1226>
                    var_256 = wp::add(var_237, var_250);
                    // local_hi += r_hi                                                           <L 1227>
                    var_257 = wp::add(var_239, var_251);
                    // local_mid += r_mid                                                         <L 1228>
                    var_258 = wp::add(var_241, var_252);
                    wp::assign(var_237, var_256);
                    wp::assign(var_239, var_257);
                    wp::assign(var_241, var_258);
                    goto start_for_13;
                end_for_13:;
                // local_combined = wp.mat33(                                                     <L 1231>
                // local_lo[0],                                                                   <L 1232>
                var_260 = wp::extract(var_237, var_259);
                // local_hi[0],                                                                   <L 1233>
                var_262 = wp::extract(var_239, var_261);
                // local_mid[0],                                                                  <L 1234>
                var_264 = wp::extract(var_241, var_263);
                // local_lo[1],                                                                   <L 1235>
                var_266 = wp::extract(var_237, var_265);
                // local_hi[1],                                                                   <L 1236>
                var_268 = wp::extract(var_239, var_267);
                // local_mid[1],                                                                  <L 1237>
                var_270 = wp::extract(var_241, var_269);
                // local_lo[2],                                                                   <L 1238>
                var_272 = wp::extract(var_237, var_271);
                // local_hi[2],                                                                   <L 1239>
                var_274 = wp::extract(var_239, var_273);
                // local_mid[2],                                                                  <L 1240>
                var_276 = wp::extract(var_241, var_275);
                var_277 = wp::mat_t<3, 3, wp::float32>(var_260, var_262, var_264, var_266, var_268, var_270, var_272, var_274, var_276);
                // combined_tile = wp.tile(local_combined, preserve_type=True)                    <L 1245>
                var_279 = wp::tile<wp::mat_t<3, 3, wp::float32>>(var_277);
                // combined_sum = wp.tile_reduce(wp.add, combined_tile)                           <L 1246>
                var_280 = wp::tile_reduce(wp::add, var_279);
                // result = combined_sum[0]                                                       <L 1247>
                var_282 = wp::tile_extract(var_280, var_281);
                // gauss_lo, gauss_hi, gauss_mid = _eval_pt_3alphas(ctx_quad_gauss, lo_next_alpha, hi_next_alpha, mid_alpha)       <L 1250>
                _eval_pt_3alphas_0(var_118, var_226, var_232, var_235, var_283, var_284, var_285);
                // lo_next = gauss_lo + wp.vec3(result[0, 0], result[1, 0], result[2, 0])         <L 1251>
                var_288 = wp::extract(var_282, var_286, var_287);
                var_291 = wp::extract(var_282, var_289, var_290);
                var_294 = wp::extract(var_282, var_292, var_293);
                var_295 = wp::vec_t<3, wp::float32>(var_288, var_291, var_294);
                var_296 = wp::add(var_283, var_295);
                // hi_next = gauss_hi + wp.vec3(result[0, 1], result[1, 1], result[2, 1])         <L 1252>
                var_299 = wp::extract(var_282, var_297, var_298);
                var_302 = wp::extract(var_282, var_300, var_301);
                var_305 = wp::extract(var_282, var_303, var_304);
                var_306 = wp::vec_t<3, wp::float32>(var_299, var_302, var_305);
                var_307 = wp::add(var_284, var_306);
                // mid = gauss_mid + wp.vec3(result[0, 2], result[1, 2], result[2, 2])            <L 1253>
                var_310 = wp::extract(var_282, var_308, var_309);
                var_313 = wp::extract(var_282, var_311, var_312);
                var_316 = wp::extract(var_282, var_314, var_315);
                var_317 = wp::vec_t<3, wp::float32>(var_310, var_313, var_316);
                var_318 = wp::add(var_285, var_317);
                // swap_lo_lo_next = _in_bracket(lo, lo_next)                                     <L 1257>
                var_319 = _in_bracket_0(var_212, var_296);
                // lo = wp.where(swap_lo_lo_next, lo_next, lo)                                    <L 1258>
                var_320 = wp::where(var_319, var_296, var_212);
                // lo_alpha = wp.where(swap_lo_lo_next, lo_next_alpha, lo_alpha)                  <L 1259>
                var_321 = wp::where(var_319, var_226, var_214);
                // swap_lo_mid = _in_bracket(lo, mid)                                             <L 1260>
                var_322 = _in_bracket_0(var_320, var_318);
                // lo = wp.where(swap_lo_mid, mid, lo)                                            <L 1261>
                var_323 = wp::where(var_322, var_318, var_320);
                // lo_alpha = wp.where(swap_lo_mid, mid_alpha, lo_alpha)                          <L 1262>
                var_324 = wp::where(var_322, var_235, var_321);
                // swap_lo_hi_next = _in_bracket(lo, hi_next)                                     <L 1263>
                var_325 = _in_bracket_0(var_323, var_307);
                // lo = wp.where(swap_lo_hi_next, hi_next, lo)                                    <L 1264>
                var_326 = wp::where(var_325, var_307, var_323);
                // lo_alpha = wp.where(swap_lo_hi_next, hi_next_alpha, lo_alpha)                  <L 1265>
                var_327 = wp::where(var_325, var_232, var_324);
                // swap_lo = swap_lo_lo_next or swap_lo_mid or swap_lo_hi_next                    <L 1266>
                var_328 = var_319;
                if (!var_328) {
                    var_328 = var_328 || var_322;
                }
                if (!var_328) {
                    var_328 = var_328 || var_325;
                }
                // swap_hi_hi_next = _in_bracket(hi, hi_next)                                     <L 1269>
                var_329 = _in_bracket_0(var_215, var_307);
                // hi = wp.where(swap_hi_hi_next, hi_next, hi)                                    <L 1270>
                var_330 = wp::where(var_329, var_307, var_215);
                // hi_alpha = wp.where(swap_hi_hi_next, hi_next_alpha, hi_alpha)                  <L 1271>
                var_331 = wp::where(var_329, var_232, var_217);
                // swap_hi_mid = _in_bracket(hi, mid)                                             <L 1272>
                var_332 = _in_bracket_0(var_330, var_318);
                // hi = wp.where(swap_hi_mid, mid, hi)                                            <L 1273>
                var_333 = wp::where(var_332, var_318, var_330);
                // hi_alpha = wp.where(swap_hi_mid, mid_alpha, hi_alpha)                          <L 1274>
                var_334 = wp::where(var_332, var_235, var_331);
                // swap_hi_lo_next = _in_bracket(hi, lo_next)                                     <L 1275>
                var_335 = _in_bracket_0(var_333, var_296);
                // hi = wp.where(swap_hi_lo_next, lo_next, hi)                                    <L 1276>
                var_336 = wp::where(var_335, var_296, var_333);
                // hi_alpha = wp.where(swap_hi_lo_next, lo_next_alpha, hi_alpha)                  <L 1277>
                var_337 = wp::where(var_335, var_226, var_334);
                // swap_hi = swap_hi_hi_next or swap_hi_mid or swap_hi_lo_next                    <L 1278>
                var_338 = var_329;
                if (!var_338) {
                    var_338 = var_338 || var_332;
                }
                if (!var_338) {
                    var_338 = var_338 || var_335;
                }
                // ls_done = (                                                                    <L 1281>
                // (not swap_lo and not swap_hi)                                                  <L 1282>
                var_341 = wp::unot(var_328);
                var_340 = var_341;
                if (var_340) {
                    var_342 = wp::unot(var_338);
                    var_340 = var_340 && var_342;
                }
                var_339 = var_340;
                if (!var_339) {
                    // or (lo[0] < 0.0 and lo[1] < 0.0 and lo[1] > -gtol)                         <L 1283>
                    var_345 = wp::extract(var_326, var_344);
                    var_347 = (var_345 < var_346);
                    var_343 = var_347;
                    if (var_343) {
                        var_349 = wp::extract(var_326, var_348);
                        var_351 = (var_349 < var_350);
                        var_343 = var_343 && var_351;
                    }
                    if (var_343) {
                        var_353 = wp::extract(var_326, var_352);
                        var_354 = wp::neg(var_68);
                        var_355 = (var_353 > var_354);
                        var_343 = var_343 && var_355;
                    }
                    var_339 = var_339 || var_343;
                }
                if (!var_339) {
                    // or (hi[0] < 0.0 and hi[1] > 0.0 and hi[1] < gtol)                          <L 1284>
                    var_358 = wp::extract(var_336, var_357);
                    var_360 = (var_358 < var_359);
                    var_356 = var_360;
                    if (var_356) {
                        var_362 = wp::extract(var_336, var_361);
                        var_364 = (var_362 > var_363);
                        var_356 = var_356 && var_364;
                    }
                    if (var_356) {
                        var_366 = wp::extract(var_336, var_365);
                        var_367 = (var_366 < var_68);
                        var_356 = var_356 && var_367;
                    }
                    var_339 = var_339 || var_356;
                }
                // improved = lo[0] < 0.0 or hi[0] < 0.0                                          <L 1288>
                var_370 = wp::extract(var_326, var_369);
                var_372 = (var_370 < var_371);
                var_368 = var_372;
                if (!var_368) {
                    var_374 = wp::extract(var_336, var_373);
                    var_376 = (var_374 < var_375);
                    var_368 = var_368 || var_376;
                }
                // lo_better = lo[0] < hi[0]                                                      <L 1289>
                var_378 = wp::extract(var_326, var_377);
                var_380 = wp::extract(var_336, var_379);
                var_381 = (var_378 < var_380);
                // best_alpha = wp.where(lo_better, lo_alpha, hi_alpha)                           <L 1290>
                var_382 = wp::where(var_381, var_327, var_337);
                // best_delta = wp.where(lo_better, lo[0], hi[0])                                 <L 1291>
                var_384 = wp::extract(var_326, var_383);
                var_386 = wp::extract(var_336, var_385);
                var_387 = wp::where(var_381, var_384, var_386);
                // alpha = wp.where(improved, best_alpha, alpha)                                  <L 1292>
                var_388 = wp::where(var_368, var_382, var_204);
                // improvement = wp.where(improved, -best_delta, improvement)                     <L 1293>
                var_389 = wp::neg(var_387);
                var_390 = wp::where(var_368, var_389, var_206);
                // if ls_done:                                                                    <L 1295>
                if (var_339) {
                    // break                                                                      <L 1296>
                    wp::assign(var_175, var_244);
                    wp::assign(var_204, var_388);
                    wp::assign(var_206, var_390);
                    wp::assign(var_212, var_326);
                    wp::assign(var_214, var_327);
                    wp::assign(var_215, var_336);
                    wp::assign(var_217, var_337);
                    goto end_for_11;
                }
                var_391 = wp::where(var_339, var_175, var_244);
                var_392 = wp::where(var_339, var_204, var_388);
                var_393 = wp::where(var_339, var_206, var_390);
                var_394 = wp::where(var_339, var_212, var_326);
                var_395 = wp::where(var_339, var_214, var_327);
                var_396 = wp::where(var_339, var_215, var_336);
                var_397 = wp::where(var_339, var_217, var_337);
                wp::assign(var_175, var_391);
                wp::assign(var_204, var_392);
                wp::assign(var_206, var_393);
                wp::assign(var_212, var_394);
                wp::assign(var_214, var_395);
                wp::assign(var_215, var_396);
                wp::assign(var_217, var_397);
                goto start_for_11;
            end_for_11:;
        }
        if (!var_202) {
            // alpha = lo_alpha_in                                                                <L 1298>
            var_398 = wp::copy(var_170);
            // improvement = -lo_in[0]                                                            <L 1299>
            var_400 = wp::extract(var_192, var_399);
            var_401 = wp::neg(var_400);
        }
        var_402 = wp::where(var_202, var_204, var_398);
        var_403 = wp::where(var_202, var_206, var_401);
        // for dofid in range(tid, nv, wp.block_dim()):                                           <L 1302>
        var_404 = builtin_block_dim();
        var_405 = wp::range(var_1, var_nv, var_404);
        start_for_15:;
            if (iter_cmp(var_405) == 0) goto end_for_15;
            var_406 = wp::iter_next(var_405);
            // qacc_out[worldid, dofid] += alpha * ctx_search_in[worldid, dofid]                  <L 1303>
            var_407 = wp::address(var_ctx_search_in, var_0, var_406);
            var_409 = wp::load(var_407);
            var_408 = wp::mul(var_402, var_409);
            var_410 = wp::atomic_add(var_qacc_out, var_0, var_406, var_408);
            // efc_Ma_out[worldid, dofid] += alpha * ctx_mv_in[worldid, dofid]                    <L 1304>
            var_411 = wp::address(var_ctx_mv_in, var_0, var_406);
            var_413 = wp::load(var_411);
            var_412 = wp::mul(var_402, var_413);
            var_414 = wp::atomic_add(var_efc_Ma_out, var_0, var_406, var_412);
            goto start_for_15;
        end_for_15:;
        // for efcid in range(tid, nefc, wp.block_dim()):                                         <L 1307>
        var_415 = builtin_block_dim();
        var_416 = wp::range(var_1, var_12, var_415);
        start_for_17:;
            if (iter_cmp(var_416) == 0) goto end_for_17;
            var_417 = wp::iter_next(var_416);
            // ctx_Jaref_out[worldid, efcid] += alpha * ctx_jv_in[worldid, efcid]                 <L 1308>
            var_418 = wp::address(var_ctx_jv_in, var_0, var_417);
            var_420 = wp::load(var_418);
            var_419 = wp::mul(var_402, var_420);
            var_421 = wp::atomic_add(var_ctx_Jaref_out, var_0, var_417, var_419);
            goto start_for_17;
        end_for_17:;
        // if tid == 0:                                                                           <L 1310>
        var_423 = (var_1 == var_422);
        if (var_423) {
            // ctx_improvement_out[worldid] = improvement                                         <L 1311>
            wp::array_store(var_ctx_improvement_out, var_0, var_403);
            // ctx_alpha_out[worldid] = alpha                                                     <L 1312>
            wp::array_store(var_ctx_alpha_out, var_0, var_402);
            // if wp.static(INCREMENTAL):                                                         <L 1313>
            // ctx_ls_exhausted_out[worldid] = wp.abs(alpha) < noise_floor                        <L 1314>
            var_425 = wp::abs(var_402);
            var_426 = (var_425 < var_164);
            wp::array_store(var_ctx_ls_exhausted_out, var_0, var_426);
        }
    }
}


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



extern "C" __global__ void _efc_contact_init__locals__kernel_6b0e3be3_cuda_kernel_forward(
    wp::launch_bounds_t<1> dim,
    wp::array_t<wp::int32> var_body_weldid,
    wp::array_t<wp::int32> var_body_dofnum,
    wp::array_t<wp::int32> var_body_dofadr,
    wp::array_t<wp::int32> var_dof_parentid,
    wp::array_t<wp::int32> var_geom_bodyid,
    wp::int32 var_njmax_in,
    wp::int32 var_njmax_nnz_in,
    wp::array_t<wp::int32> var_nacon_in,
    wp::array_t<wp::float32> var_dist_in,
    wp::array_t<wp::int32> var_condim_in,
    wp::array_t<wp::float32> var_includemargin_in,
    wp::array_t<wp::int32> var_worldid_in,
    wp::array_t<wp::vec_t<2, wp::int32>> var_geom_in,
    wp::array_t<wp::int32> var_type_in,
    wp::array_t<wp::int32> var_nefc_out,
    wp::array_t<wp::int32> var_contact_efc_address_out,
    wp::array_t<wp::int32> var_efc_id_out,
    wp::array_t<wp::int32> var_efc_jtdaj_adr_out,
    wp::array_t<wp::int32> var_efc_jtdaj_nrow_out,
    wp::array_t<wp::int32> var_efc_jtdaj_nblock_out,
    wp::array_t<wp::int32> var_efc_J_rownnz_out,
    wp::array_t<wp::int32> var_efc_J_rowadr_out,
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
        const wp::int32 var_1 = 0;
        wp::int32* var_2;
        bool var_3;
        wp::int32 var_4;
        wp::int32* var_5;
        const wp::int32 var_6 = 1;
        wp::int32 var_7;
        wp::int32 var_8;
        bool var_9;
        wp::int32* var_10;
        wp::int32 var_11;
        wp::int32 var_12;
        wp::float32* var_13;
        wp::float32 var_14;
        wp::float32 var_15;
        wp::float32* var_16;
        wp::float32 var_17;
        wp::float32 var_18;
        const wp::int32 var_19 = 0;
        bool var_20;
        bool var_21;
        const bool var_22 = false;
        const wp::int32 var_23 = 1;
        bool var_24;
        const wp::int32 var_25 = 1;
        const wp::int32 var_26 = 2;
        const wp::int32 var_27 = 1;
        wp::int32 var_28;
        wp::int32 var_29;
        wp::int32 var_30;
        wp::int32* var_31;
        wp::int32 var_32;
        wp::int32 var_33;
        wp::int32 var_34;
        wp::range_t var_35;
        wp::int32 var_36;
        wp::int32 var_37;
        bool var_38;
        const wp::int32 var_39 = -1;
        const bool var_40 = false;
        const bool var_41 = false;
        //---------
        // forward
        // def kernel(                                                                            <L 2646>
        // conid = wp.tid()                                                                       <L 2676>
        var_0 = builtin_tid1d();
        // if conid >= nacon_in[0]:                                                               <L 2678>
        var_2 = wp::address(var_nacon_in, var_1);
        var_4 = wp::load(var_2);
        var_3 = (var_0 >= var_4);
        if (var_3) {
            // return                                                                             <L 2679>
            continue;
        }
        // if not type_in[conid] & ContactType.CONSTRAINT:                                        <L 2681>
        var_5 = wp::address(var_type_in, var_0);
        var_8 = wp::load(var_5);
        var_7 = wp::bit_and(var_8, var_6);
        var_9 = wp::unot(var_7);
        if (var_9) {
            // return                                                                             <L 2682>
            continue;
        }
        // condim = condim_in[conid]                                                              <L 2684>
        var_10 = wp::address(var_condim_in, var_0);
        var_12 = wp::load(var_10);
        var_11 = wp::copy(var_12);
        // includemargin = includemargin_in[conid]                                                <L 2686>
        var_13 = wp::address(var_includemargin_in, var_0);
        var_15 = wp::load(var_13);
        var_14 = wp::copy(var_15);
        // pos = dist_in[conid] - includemargin                                                   <L 2687>
        var_16 = wp::address(var_dist_in, var_0);
        var_18 = wp::load(var_16);
        var_17 = wp::sub(var_18, var_14);
        // active = pos < 0                                                                       <L 2688>
        var_20 = (var_17 < var_19);
        // if not active:                                                                         <L 2690>
        var_21 = wp::unot(var_20);
        if (var_21) {
            // return                                                                             <L 2691>
            continue;
        }
        // if wp.static(IS_ELLIPTIC):                                                             <L 2693>
        // if condim == 1:                                                                        <L 2696>
        var_24 = (var_11 == var_23);
        if (var_24) {
            // ndim = 1                                                                           <L 2697>
        }
        if (!var_24) {
            // ndim = 2 * (condim - 1)                                                            <L 2699>
            var_28 = wp::sub(var_11, var_27);
            var_29 = wp::mul(var_26, var_28);
        }
        var_30 = wp::where(var_24, var_25, var_29);
        // worldid = worldid_in[conid]                                                            <L 2701>
        var_31 = wp::address(var_worldid_in, var_0);
        var_33 = wp::load(var_31);
        var_32 = wp::copy(var_33);
        // base_efcid = wp.atomic_add(nefc_out, worldid, ndim)                                    <L 2704>
        var_34 = wp::atomic_add(var_nefc_out, var_32, var_30);
        // for dim in range(ndim):                                                                <L 2705>
        var_35 = wp::range(var_30);
        start_for_3:;
            if (iter_cmp(var_35) == 0) goto end_for_3;
            var_36 = wp::iter_next(var_35);
            // efcid = base_efcid + dim                                                           <L 2706>
            var_37 = wp::add(var_34, var_36);
            // if efcid >= njmax_in:                                                              <L 2707>
            var_38 = (var_37 >= var_njmax_in);
            if (var_38) {
                // contact_efc_address_out[conid, dim] = -1                                       <L 2708>
                wp::array_store(var_contact_efc_address_out, var_0, var_36, var_39);
            }
            if (!var_38) {
                // contact_efc_address_out[conid, dim] = efcid                                    <L 2710>
                wp::array_store(var_contact_efc_address_out, var_0, var_36, var_37);
                // efc_id_out[worldid, efcid] = conid                                             <L 2712>
                wp::array_store(var_efc_id_out, var_32, var_37, var_0);
            }
            goto start_for_3;
        end_for_3:;
        // if wp.static(is_sparse and newton):                                                    <L 2714>
        // if wp.static(IS_SPARSE):                                                               <L 2720>
    }
}


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


struct Geom_3242f8a8
{
    wp::vec_t<3, wp::float32> pos;
    wp::mat_t<3, 3, wp::float32> rot;
    wp::vec_t<3, wp::float32> normal;
    wp::vec_t<3, wp::float32> size;
    wp::float32 margin;
    wp::mat_t<6, 3, wp::float32> hfprism;
    wp::int32 vertadr;
    wp::int32 vertnum;
    wp::array_t<wp::vec_t<3, wp::float32>> vert;
    wp::int32 graphadr;
    wp::array_t<wp::int32> graph;
    wp::int32 mesh_polynum;
    wp::int32 mesh_polyadr;
    wp::array_t<wp::vec_t<3, wp::float32>> mesh_polynormal;
    wp::array_t<wp::int32> mesh_polyvertadr;
    wp::array_t<wp::int32> mesh_polyvertnum;
    wp::array_t<wp::int32> mesh_polyvert;
    wp::array_t<wp::int32> mesh_polymapadr;
    wp::array_t<wp::int32> mesh_polymapnum;
    wp::array_t<wp::int32> mesh_polymap;
    wp::int32 index;


    Geom_3242f8a8() = default;
    CUDA_CALLABLE Geom_3242f8a8(wp::vec_t<3, wp::float32> const& pos,
    wp::mat_t<3, 3, wp::float32> const& rot = {},
    wp::vec_t<3, wp::float32> const& normal = {},
    wp::vec_t<3, wp::float32> const& size = {},
    wp::float32 const& margin = {},
    wp::mat_t<6, 3, wp::float32> const& hfprism = {},
    wp::int32 const& vertadr = {},
    wp::int32 const& vertnum = {},
    wp::array_t<wp::vec_t<3, wp::float32>> const& vert = {},
    wp::int32 const& graphadr = {},
    wp::array_t<wp::int32> const& graph = {},
    wp::int32 const& mesh_polynum = {},
    wp::int32 const& mesh_polyadr = {},
    wp::array_t<wp::vec_t<3, wp::float32>> const& mesh_polynormal = {},
    wp::array_t<wp::int32> const& mesh_polyvertadr = {},
    wp::array_t<wp::int32> const& mesh_polyvertnum = {},
    wp::array_t<wp::int32> const& mesh_polyvert = {},
    wp::array_t<wp::int32> const& mesh_polymapadr = {},
    wp::array_t<wp::int32> const& mesh_polymapnum = {},
    wp::array_t<wp::int32> const& mesh_polymap = {},
    wp::int32 const& index = {})
        : pos{pos}
        , rot{rot}
        , normal{normal}
        , size{size}
        , margin{margin}
        , hfprism{hfprism}
        , vertadr{vertadr}
        , vertnum{vertnum}
        , vert{vert}
        , graphadr{graphadr}
        , graph{graph}
        , mesh_polynum{mesh_polynum}
        , mesh_polyadr{mesh_polyadr}
        , mesh_polynormal{mesh_polynormal}
        , mesh_polyvertadr{mesh_polyvertadr}
        , mesh_polyvertnum{mesh_polyvertnum}
        , mesh_polyvert{mesh_polyvert}
        , mesh_polymapadr{mesh_polymapadr}
        , mesh_polymapnum{mesh_polymapnum}
        , mesh_polymap{mesh_polymap}
        , index{index}

    {
    }

    CUDA_CALLABLE Geom_3242f8a8& operator += (const Geom_3242f8a8& rhs)
    {    pos += rhs.pos;
    rot += rhs.rot;
    normal += rhs.normal;
    size += rhs.size;
    margin += rhs.margin;
    hfprism += rhs.hfprism;
    vertadr += rhs.vertadr;
    vertnum += rhs.vertnum;
    graphadr += rhs.graphadr;
    mesh_polynum += rhs.mesh_polynum;
    mesh_polyadr += rhs.mesh_polyadr;
    index += rhs.index;

        return *this;}

};

static CUDA_CALLABLE void adj_Geom_3242f8a8(wp::vec_t<3, wp::float32> const&,
    wp::mat_t<3, 3, wp::float32> const&,
    wp::vec_t<3, wp::float32> const&,
    wp::vec_t<3, wp::float32> const&,
    wp::float32 const&,
    wp::mat_t<6, 3, wp::float32> const&,
    wp::int32 const&,
    wp::int32 const&,
    wp::array_t<wp::vec_t<3, wp::float32>> const&,
    wp::int32 const&,
    wp::array_t<wp::int32> const&,
    wp::int32 const&,
    wp::int32 const&,
    wp::array_t<wp::vec_t<3, wp::float32>> const&,
    wp::array_t<wp::int32> const&,
    wp::array_t<wp::int32> const&,
    wp::array_t<wp::int32> const&,
    wp::array_t<wp::int32> const&,
    wp::array_t<wp::int32> const&,
    wp::array_t<wp::int32> const&,
    wp::int32 const&,
    wp::vec_t<3, wp::float32> & adj_pos,
    wp::mat_t<3, 3, wp::float32> & adj_rot,
    wp::vec_t<3, wp::float32> & adj_normal,
    wp::vec_t<3, wp::float32> & adj_size,
    wp::float32 & adj_margin,
    wp::mat_t<6, 3, wp::float32> & adj_hfprism,
    wp::int32 & adj_vertadr,
    wp::int32 & adj_vertnum,
    wp::array_t<wp::vec_t<3, wp::float32>> & adj_vert,
    wp::int32 & adj_graphadr,
    wp::array_t<wp::int32> & adj_graph,
    wp::int32 & adj_mesh_polynum,
    wp::int32 & adj_mesh_polyadr,
    wp::array_t<wp::vec_t<3, wp::float32>> & adj_mesh_polynormal,
    wp::array_t<wp::int32> & adj_mesh_polyvertadr,
    wp::array_t<wp::int32> & adj_mesh_polyvertnum,
    wp::array_t<wp::int32> & adj_mesh_polyvert,
    wp::array_t<wp::int32> & adj_mesh_polymapadr,
    wp::array_t<wp::int32> & adj_mesh_polymapnum,
    wp::array_t<wp::int32> & adj_mesh_polymap,
    wp::int32 & adj_index,
    Geom_3242f8a8 & adj_ret)
{
    adj_pos += adj_ret.pos;
    adj_rot += adj_ret.rot;
    adj_normal += adj_ret.normal;
    adj_size += adj_ret.size;
    adj_margin += adj_ret.margin;
    adj_hfprism += adj_ret.hfprism;
    adj_vertadr += adj_ret.vertadr;
    adj_vertnum += adj_ret.vertnum;
    adj_vert = adj_ret.vert;
    adj_graphadr += adj_ret.graphadr;
    adj_graph = adj_ret.graph;
    adj_mesh_polynum += adj_ret.mesh_polynum;
    adj_mesh_polyadr += adj_ret.mesh_polyadr;
    adj_mesh_polynormal = adj_ret.mesh_polynormal;
    adj_mesh_polyvertadr = adj_ret.mesh_polyvertadr;
    adj_mesh_polyvertnum = adj_ret.mesh_polyvertnum;
    adj_mesh_polyvert = adj_ret.mesh_polyvert;
    adj_mesh_polymapadr = adj_ret.mesh_polymapadr;
    adj_mesh_polymapnum = adj_ret.mesh_polymapnum;
    adj_mesh_polymap = adj_ret.mesh_polymap;
    adj_index += adj_ret.index;
}

// Required when compiling adjoints.
CUDA_CALLABLE Geom_3242f8a8 add(const Geom_3242f8a8& a, const Geom_3242f8a8& b)
{
    return Geom_3242f8a8();
}

CUDA_CALLABLE void adj_atomic_add(Geom_3242f8a8* p, Geom_3242f8a8 t)
{
    wp::adj_atomic_add(&p->pos, t.pos);
    wp::adj_atomic_add(&p->rot, t.rot);
    wp::adj_atomic_add(&p->normal, t.normal);
    wp::adj_atomic_add(&p->size, t.size);
    wp::adj_atomic_add(&p->margin, t.margin);
    wp::adj_atomic_add(&p->hfprism, t.hfprism);
    wp::adj_atomic_add(&p->vertadr, t.vertadr);
    wp::adj_atomic_add(&p->vertnum, t.vertnum);
    wp::adj_atomic_add(&p->vert, t.vert);
    wp::adj_atomic_add(&p->graphadr, t.graphadr);
    wp::adj_atomic_add(&p->graph, t.graph);
    wp::adj_atomic_add(&p->mesh_polynum, t.mesh_polynum);
    wp::adj_atomic_add(&p->mesh_polyadr, t.mesh_polyadr);
    wp::adj_atomic_add(&p->mesh_polynormal, t.mesh_polynormal);
    wp::adj_atomic_add(&p->mesh_polyvertadr, t.mesh_polyvertadr);
    wp::adj_atomic_add(&p->mesh_polyvertnum, t.mesh_polyvertnum);
    wp::adj_atomic_add(&p->mesh_polyvert, t.mesh_polyvert);
    wp::adj_atomic_add(&p->mesh_polymapadr, t.mesh_polymapadr);
    wp::adj_atomic_add(&p->mesh_polymapnum, t.mesh_polymapnum);
    wp::adj_atomic_add(&p->mesh_polymap, t.mesh_polymap);
    wp::adj_atomic_add(&p->index, t.index);
}




// /home/cofallen/miniconda3/envs/mjlab/lib/python3.12/site-packages/mujoco_warp/_src/collision_core.py:290
static CUDA_CALLABLE void contact_margin_gap_0(
    wp::array_t<wp::float32> var_geom_margin,
    wp::array_t<wp::float32> var_geom_gap,
    wp::array_t<wp::float32> var_pair_margin,
    wp::array_t<wp::float32> var_pair_gap,
    wp::vec_t<2, wp::int32> var_geoms,
    wp::int32 var_pairid,
    wp::int32 var_worldid,
    wp::float32 & ret_0,
    wp::float32 & ret_1)
{
    //---------
    // primal vars
    const wp::int32 var_0 = -1;
    bool var_1;
    wp::shape_t* var_2;
    const wp::int32 var_3 = 0;
    wp::int32 var_4;
    wp::shape_t var_5;
    wp::int32 var_6;
    wp::float32* var_7;
    wp::float32 var_8;
    wp::float32 var_9;
    wp::shape_t* var_10;
    const wp::int32 var_11 = 0;
    wp::int32 var_12;
    wp::shape_t var_13;
    wp::int32 var_14;
    wp::float32* var_15;
    wp::float32 var_16;
    wp::float32 var_17;
    const wp::int32 var_18 = 0;
    wp::int32 var_19;
    const wp::int32 var_20 = 1;
    wp::int32 var_21;
    wp::shape_t* var_22;
    const wp::int32 var_23 = 0;
    wp::int32 var_24;
    wp::shape_t var_25;
    wp::int32 var_26;
    wp::shape_t* var_27;
    const wp::int32 var_28 = 0;
    wp::int32 var_29;
    wp::shape_t var_30;
    wp::int32 var_31;
    wp::float32* var_32;
    wp::float32* var_33;
    wp::float32 var_34;
    wp::float32 var_35;
    wp::float32 var_36;
    wp::float32* var_37;
    wp::float32* var_38;
    wp::float32 var_39;
    wp::float32 var_40;
    wp::float32 var_41;
    wp::float32 var_42;
    wp::float32 var_43;
    //---------
    // forward
    // def contact_margin_gap(                                                                <L 291>
    // if pairid > -1:                                                                        <L 302>
    var_1 = (var_pairid > var_0);
    if (var_1) {
        // margin = pair_margin[worldid % pair_margin.shape[0], pairid]                       <L 303>
        var_2 = &(var_pair_margin.shape);
        var_5 = wp::load(var_2);
        var_4 = wp::extract(var_5, var_3);
        var_6 = wp::mod(var_worldid, var_4);
        var_7 = wp::address(var_pair_margin, var_6, var_pairid);
        var_9 = wp::load(var_7);
        var_8 = wp::copy(var_9);
        // gap = pair_gap[worldid % pair_gap.shape[0], pairid]                                <L 304>
        var_10 = &(var_pair_gap.shape);
        var_13 = wp::load(var_10);
        var_12 = wp::extract(var_13, var_11);
        var_14 = wp::mod(var_worldid, var_12);
        var_15 = wp::address(var_pair_gap, var_14, var_pairid);
        var_17 = wp::load(var_15);
        var_16 = wp::copy(var_17);
    }
    if (!var_1) {
        // g1 = geoms[0]                                                                      <L 306>
        var_19 = wp::extract(var_geoms, var_18);
        // g2 = geoms[1]                                                                      <L 307>
        var_21 = wp::extract(var_geoms, var_20);
        // margin_id = worldid % geom_margin.shape[0]                                         <L 308>
        var_22 = &(var_geom_margin.shape);
        var_25 = wp::load(var_22);
        var_24 = wp::extract(var_25, var_23);
        var_26 = wp::mod(var_worldid, var_24);
        // gap_id = worldid % geom_gap.shape[0]                                               <L 309>
        var_27 = &(var_geom_gap.shape);
        var_30 = wp::load(var_27);
        var_29 = wp::extract(var_30, var_28);
        var_31 = wp::mod(var_worldid, var_29);
        // margin = geom_margin[margin_id, g1] + geom_margin[margin_id, g2]                   <L 310>
        var_32 = wp::address(var_geom_margin, var_26, var_19);
        var_33 = wp::address(var_geom_margin, var_26, var_21);
        var_35 = wp::load(var_32);
        var_36 = wp::load(var_33);
        var_34 = wp::add(var_35, var_36);
        // gap = geom_gap[gap_id, g1] + geom_gap[gap_id, g2]                                  <L 311>
        var_37 = wp::address(var_geom_gap, var_31, var_19);
        var_38 = wp::address(var_geom_gap, var_31, var_21);
        var_40 = wp::load(var_37);
        var_41 = wp::load(var_38);
        var_39 = wp::add(var_40, var_41);
    }
    var_42 = wp::where(var_1, var_8, var_34);
    var_43 = wp::where(var_1, var_16, var_39);
    // return margin, gap                                                                     <L 313>
    ret_0 = var_42;
    ret_1 = var_43;
    return;
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


// /home/cofallen/miniconda3/envs/mjlab/lib/python3.12/site-packages/mujoco_warp/_src/collision_core.py:316
static CUDA_CALLABLE void contact_material_params_0(
    wp::array_t<wp::int32> var_geom_condim,
    wp::array_t<wp::int32> var_geom_priority,
    wp::array_t<wp::float32> var_geom_solmix,
    wp::array_t<wp::vec_t<2, wp::float32>> var_geom_solref,
    wp::array_t<wp::vec_t<5, wp::float32>> var_geom_solimp,
    wp::array_t<wp::vec_t<3, wp::float32>> var_geom_friction,
    wp::array_t<wp::int32> var_pair_dim,
    wp::array_t<wp::vec_t<2, wp::float32>> var_pair_solref,
    wp::array_t<wp::vec_t<2, wp::float32>> var_pair_solreffriction,
    wp::array_t<wp::vec_t<5, wp::float32>> var_pair_solimp,
    wp::array_t<wp::vec_t<5, wp::float32>> var_pair_friction,
    wp::vec_t<2, wp::int32> var_geoms,
    wp::int32 var_pairid,
    wp::int32 var_worldid,
    wp::int32 & ret_0,
    wp::vec_t<5, wp::float32> & ret_1,
    wp::vec_t<2, wp::float32> & ret_2,
    wp::vec_t<2, wp::float32> & ret_3,
    wp::vec_t<5, wp::float32> & ret_4)
{
    //---------
    // primal vars
    const wp::int32 var_0 = -1;
    bool var_1;
    wp::int32* var_2;
    wp::int32 var_3;
    wp::int32 var_4;
    wp::shape_t* var_5;
    const wp::int32 var_6 = 0;
    wp::int32 var_7;
    wp::shape_t var_8;
    wp::int32 var_9;
    wp::vec_t<5, wp::float32>* var_10;
    wp::vec_t<5, wp::float32> var_11;
    wp::vec_t<5, wp::float32> var_12;
    wp::shape_t* var_13;
    const wp::int32 var_14 = 0;
    wp::int32 var_15;
    wp::shape_t var_16;
    wp::int32 var_17;
    wp::vec_t<2, wp::float32>* var_18;
    wp::vec_t<2, wp::float32> var_19;
    wp::vec_t<2, wp::float32> var_20;
    wp::shape_t* var_21;
    const wp::int32 var_22 = 0;
    wp::int32 var_23;
    wp::shape_t var_24;
    wp::int32 var_25;
    wp::vec_t<2, wp::float32>* var_26;
    wp::vec_t<2, wp::float32> var_27;
    wp::vec_t<2, wp::float32> var_28;
    wp::shape_t* var_29;
    const wp::int32 var_30 = 0;
    wp::int32 var_31;
    wp::shape_t var_32;
    wp::int32 var_33;
    wp::vec_t<5, wp::float32>* var_34;
    wp::vec_t<5, wp::float32> var_35;
    wp::vec_t<5, wp::float32> var_36;
    const wp::int32 var_37 = 0;
    wp::int32 var_38;
    const wp::int32 var_39 = 1;
    wp::int32 var_40;
    wp::shape_t* var_41;
    const wp::int32 var_42 = 0;
    wp::int32 var_43;
    wp::shape_t var_44;
    wp::int32 var_45;
    wp::shape_t* var_46;
    const wp::int32 var_47 = 0;
    wp::int32 var_48;
    wp::shape_t var_49;
    wp::int32 var_50;
    wp::shape_t* var_51;
    const wp::int32 var_52 = 0;
    wp::int32 var_53;
    wp::shape_t var_54;
    wp::int32 var_55;
    wp::shape_t* var_56;
    const wp::int32 var_57 = 0;
    wp::int32 var_58;
    wp::shape_t var_59;
    wp::int32 var_60;
    wp::float32* var_61;
    wp::float32 var_62;
    wp::float32 var_63;
    wp::float32* var_64;
    wp::float32 var_65;
    wp::float32 var_66;
    wp::int32* var_67;
    wp::int32 var_68;
    wp::int32 var_69;
    wp::int32* var_70;
    wp::int32 var_71;
    wp::int32 var_72;
    wp::int32* var_73;
    wp::int32 var_74;
    wp::int32 var_75;
    wp::int32* var_76;
    wp::int32 var_77;
    wp::int32 var_78;
    bool var_79;
    const wp::float32 var_80 = 1.0;
    wp::int32 var_81;
    wp::vec_t<3, wp::float32>* var_82;
    wp::vec_t<3, wp::float32> var_83;
    wp::vec_t<3, wp::float32> var_84;
    bool var_85;
    const wp::float32 var_86 = 0.0;
    wp::int32 var_87;
    wp::vec_t<3, wp::float32>* var_88;
    wp::vec_t<3, wp::float32> var_89;
    wp::vec_t<3, wp::float32> var_90;
    wp::float32 var_91;
    wp::float32 var_92;
    bool var_93;
    const wp::float32 var_94 = 1e-15;
    bool var_95;
    bool var_96;
    const wp::float32 var_97 = 0.5;
    wp::float32 var_98;
    bool var_99;
    bool var_100;
    bool var_101;
    const wp::float32 var_102 = 0.0;
    wp::float32 var_103;
    bool var_104;
    bool var_105;
    bool var_106;
    const wp::float32 var_107 = 1.0;
    wp::float32 var_108;
    wp::int32 var_109;
    wp::vec_t<3, wp::float32>* var_110;
    wp::vec_t<3, wp::float32>* var_111;
    wp::vec_t<3, wp::float32> var_112;
    wp::vec_t<3, wp::float32> var_113;
    wp::vec_t<3, wp::float32> var_114;
    wp::float32 var_115;
    wp::int32 var_116;
    wp::vec_t<3, wp::float32> var_117;
    wp::float32 var_118;
    wp::int32 var_119;
    wp::vec_t<3, wp::float32> var_120;
    const wp::int32 var_121 = 0;
    wp::float32 var_122;
    const wp::int32 var_123 = 0;
    wp::float32 var_124;
    const wp::int32 var_125 = 1;
    wp::float32 var_126;
    const wp::int32 var_127 = 2;
    wp::float32 var_128;
    const wp::int32 var_129 = 2;
    wp::float32 var_130;
    wp::vec_t<5, wp::float32> var_131;
    bool var_132;
    wp::vec_t<2, wp::float32>* var_133;
    const wp::int32 var_134 = 0;
    wp::float32 var_135;
    wp::vec_t<2, wp::float32> var_136;
    const wp::float32 var_137 = 0.0;
    bool var_138;
    wp::vec_t<2, wp::float32>* var_139;
    const wp::int32 var_140 = 0;
    wp::float32 var_141;
    wp::vec_t<2, wp::float32> var_142;
    const wp::float32 var_143 = 0.0;
    bool var_144;
    wp::vec_t<2, wp::float32>* var_145;
    wp::vec_t<2, wp::float32> var_146;
    wp::vec_t<2, wp::float32> var_147;
    const wp::float32 var_148 = 1.0;
    wp::float32 var_149;
    wp::vec_t<2, wp::float32>* var_150;
    wp::vec_t<2, wp::float32> var_151;
    wp::vec_t<2, wp::float32> var_152;
    wp::vec_t<2, wp::float32> var_153;
    wp::vec_t<2, wp::float32>* var_154;
    wp::vec_t<2, wp::float32>* var_155;
    wp::vec_t<2, wp::float32> var_156;
    wp::vec_t<2, wp::float32> var_157;
    wp::vec_t<2, wp::float32> var_158;
    wp::vec_t<2, wp::float32> var_159;
    const wp::float32 var_160 = 0.0;
    const wp::float32 var_161 = 0.0;
    wp::vec_t<2, wp::float32> var_162;
    wp::vec_t<5, wp::float32>* var_163;
    wp::vec_t<5, wp::float32> var_164;
    wp::vec_t<5, wp::float32> var_165;
    const wp::float32 var_166 = 1.0;
    wp::float32 var_167;
    wp::vec_t<5, wp::float32>* var_168;
    wp::vec_t<5, wp::float32> var_169;
    wp::vec_t<5, wp::float32> var_170;
    wp::vec_t<5, wp::float32> var_171;
    wp::int32 var_172;
    wp::vec_t<5, wp::float32> var_173;
    wp::vec_t<2, wp::float32> var_174;
    wp::vec_t<2, wp::float32> var_175;
    wp::vec_t<5, wp::float32> var_176;
    const wp::float32 var_177 = 1e-05;
    const wp::int32 var_178 = 0;
    wp::float32 var_179;
    wp::float32 var_180;
    const wp::int32 var_181 = 1;
    wp::float32 var_182;
    wp::float32 var_183;
    const wp::int32 var_184 = 2;
    wp::float32 var_185;
    wp::float32 var_186;
    const wp::int32 var_187 = 3;
    wp::float32 var_188;
    wp::float32 var_189;
    const wp::int32 var_190 = 4;
    wp::float32 var_191;
    wp::float32 var_192;
    wp::vec_t<5, wp::float32> var_193;
    //---------
    // forward
    // def contact_material_params(                                                           <L 317>
    // if pairid > -1:                                                                        <L 335>
    var_1 = (var_pairid > var_0);
    if (var_1) {
        // condim = pair_dim[pairid]                                                          <L 336>
        var_2 = wp::address(var_pair_dim, var_pairid);
        var_4 = wp::load(var_2);
        var_3 = wp::copy(var_4);
        // friction = pair_friction[worldid % pair_friction.shape[0], pairid]                 <L 337>
        var_5 = &(var_pair_friction.shape);
        var_8 = wp::load(var_5);
        var_7 = wp::extract(var_8, var_6);
        var_9 = wp::mod(var_worldid, var_7);
        var_10 = wp::address(var_pair_friction, var_9, var_pairid);
        var_12 = wp::load(var_10);
        var_11 = wp::copy(var_12);
        // solref = pair_solref[worldid % pair_solref.shape[0], pairid]                       <L 338>
        var_13 = &(var_pair_solref.shape);
        var_16 = wp::load(var_13);
        var_15 = wp::extract(var_16, var_14);
        var_17 = wp::mod(var_worldid, var_15);
        var_18 = wp::address(var_pair_solref, var_17, var_pairid);
        var_20 = wp::load(var_18);
        var_19 = wp::copy(var_20);
        // solreffriction = pair_solreffriction[worldid % pair_solreffriction.shape[0], pairid]       <L 339>
        var_21 = &(var_pair_solreffriction.shape);
        var_24 = wp::load(var_21);
        var_23 = wp::extract(var_24, var_22);
        var_25 = wp::mod(var_worldid, var_23);
        var_26 = wp::address(var_pair_solreffriction, var_25, var_pairid);
        var_28 = wp::load(var_26);
        var_27 = wp::copy(var_28);
        // solimp = pair_solimp[worldid % pair_solimp.shape[0], pairid]                       <L 340>
        var_29 = &(var_pair_solimp.shape);
        var_32 = wp::load(var_29);
        var_31 = wp::extract(var_32, var_30);
        var_33 = wp::mod(var_worldid, var_31);
        var_34 = wp::address(var_pair_solimp, var_33, var_pairid);
        var_36 = wp::load(var_34);
        var_35 = wp::copy(var_36);
    }
    if (!var_1) {
        // g1 = geoms[0]                                                                      <L 342>
        var_38 = wp::extract(var_geoms, var_37);
        // g2 = geoms[1]                                                                      <L 343>
        var_40 = wp::extract(var_geoms, var_39);
        // solmix_id = worldid % geom_solmix.shape[0]                                         <L 344>
        var_41 = &(var_geom_solmix.shape);
        var_44 = wp::load(var_41);
        var_43 = wp::extract(var_44, var_42);
        var_45 = wp::mod(var_worldid, var_43);
        // friction_id = worldid % geom_friction.shape[0]                                     <L 345>
        var_46 = &(var_geom_friction.shape);
        var_49 = wp::load(var_46);
        var_48 = wp::extract(var_49, var_47);
        var_50 = wp::mod(var_worldid, var_48);
        // solref_id = worldid % geom_solref.shape[0]                                         <L 346>
        var_51 = &(var_geom_solref.shape);
        var_54 = wp::load(var_51);
        var_53 = wp::extract(var_54, var_52);
        var_55 = wp::mod(var_worldid, var_53);
        // solimp_id = worldid % geom_solimp.shape[0]                                         <L 347>
        var_56 = &(var_geom_solimp.shape);
        var_59 = wp::load(var_56);
        var_58 = wp::extract(var_59, var_57);
        var_60 = wp::mod(var_worldid, var_58);
        // solmix1 = geom_solmix[solmix_id, g1]                                               <L 349>
        var_61 = wp::address(var_geom_solmix, var_45, var_38);
        var_63 = wp::load(var_61);
        var_62 = wp::copy(var_63);
        // solmix2 = geom_solmix[solmix_id, g2]                                               <L 350>
        var_64 = wp::address(var_geom_solmix, var_45, var_40);
        var_66 = wp::load(var_64);
        var_65 = wp::copy(var_66);
        // condim1 = geom_condim[g1]                                                          <L 352>
        var_67 = wp::address(var_geom_condim, var_38);
        var_69 = wp::load(var_67);
        var_68 = wp::copy(var_69);
        // condim2 = geom_condim[g2]                                                          <L 353>
        var_70 = wp::address(var_geom_condim, var_40);
        var_72 = wp::load(var_70);
        var_71 = wp::copy(var_72);
        // p1 = geom_priority[g1]                                                             <L 356>
        var_73 = wp::address(var_geom_priority, var_38);
        var_75 = wp::load(var_73);
        var_74 = wp::copy(var_75);
        // p2 = geom_priority[g2]                                                             <L 357>
        var_76 = wp::address(var_geom_priority, var_40);
        var_78 = wp::load(var_76);
        var_77 = wp::copy(var_78);
        // if p1 > p2:                                                                        <L 359>
        var_79 = (var_74 > var_77);
        if (var_79) {
            // mix = 1.0                                                                      <L 360>
            // condim = condim1                                                               <L 361>
            var_81 = wp::copy(var_68);
            // max_geom_friction = geom_friction[friction_id, g1]                             <L 362>
            var_82 = wp::address(var_geom_friction, var_50, var_38);
            var_84 = wp::load(var_82);
            var_83 = wp::copy(var_84);
        }
        if (!var_79) {
            // elif p2 > p1:                                                                  <L 363>
            var_85 = (var_77 > var_74);
            if (var_85) {
                // mix = 0.0                                                                  <L 364>
                // condim = condim2                                                           <L 365>
                var_87 = wp::copy(var_71);
                // max_geom_friction = geom_friction[friction_id, g2]                         <L 366>
                var_88 = wp::address(var_geom_friction, var_50, var_40);
                var_90 = wp::load(var_88);
                var_89 = wp::copy(var_90);
            }
            if (!var_85) {
                // mix = safe_div(solmix1, solmix1 + solmix2)                                 <L 368>
                var_91 = wp::add(var_62, var_65);
                var_92 = safe_div_0(var_62, var_91);
                // mix = wp.where((solmix1 < MJ_MINVAL) and (solmix2 < MJ_MINVAL), 0.5, mix)       <L 369>
                var_95 = (var_62 < var_94);
                var_93 = var_95;
                if (var_93) {
                    var_96 = (var_65 < var_94);
                    var_93 = var_93 && var_96;
                }
                var_98 = wp::where(var_93, var_97, var_92);
                // mix = wp.where((solmix1 < MJ_MINVAL) and (solmix2 >= MJ_MINVAL), 0.0, mix)       <L 370>
                var_100 = (var_62 < var_94);
                var_99 = var_100;
                if (var_99) {
                    var_101 = (var_65 >= var_94);
                    var_99 = var_99 && var_101;
                }
                var_103 = wp::where(var_99, var_102, var_98);
                // mix = wp.where((solmix1 >= MJ_MINVAL) and (solmix2 < MJ_MINVAL), 1.0, mix)       <L 371>
                var_105 = (var_62 >= var_94);
                var_104 = var_105;
                if (var_104) {
                    var_106 = (var_65 < var_94);
                    var_104 = var_104 && var_106;
                }
                var_108 = wp::where(var_104, var_107, var_103);
                // condim = wp.max(condim1, condim2)                                          <L 372>
                var_109 = wp::max(var_68, var_71);
                // max_geom_friction = wp.max(geom_friction[friction_id, g1], geom_friction[friction_id, g2])       <L 373>
                var_110 = wp::address(var_geom_friction, var_50, var_38);
                var_111 = wp::address(var_geom_friction, var_50, var_40);
                var_113 = wp::load(var_110);
                var_114 = wp::load(var_111);
                var_112 = wp::max(var_113, var_114);
            }
            var_115 = wp::where(var_85, var_86, var_108);
            var_116 = wp::where(var_85, var_87, var_109);
            var_117 = wp::where(var_85, var_89, var_112);
        }
        var_118 = wp::where(var_79, var_80, var_115);
        var_119 = wp::where(var_79, var_81, var_116);
        var_120 = wp::where(var_79, var_83, var_117);
        // friction = vec5(                                                                   <L 375>
        // max_geom_friction[0],                                                              <L 376>
        var_122 = wp::extract(var_120, var_121);
        // max_geom_friction[0],                                                              <L 377>
        var_124 = wp::extract(var_120, var_123);
        // max_geom_friction[1],                                                              <L 378>
        var_126 = wp::extract(var_120, var_125);
        // max_geom_friction[2],                                                              <L 379>
        var_128 = wp::extract(var_120, var_127);
        // max_geom_friction[2],                                                              <L 380>
        var_130 = wp::extract(var_120, var_129);
        var_131 = wp::vec_t<5, wp::float32>({var_122, var_124, var_126, var_128, var_130});
        // if geom_solref[solref_id, g1][0] > 0.0 and geom_solref[solref_id, g2][0] > 0.0:       <L 383>
        var_133 = wp::address(var_geom_solref, var_55, var_38);
        var_136 = wp::load(var_133);
        var_135 = wp::extract(var_136, var_134);
        var_138 = (var_135 > var_137);
        var_132 = var_138;
        if (var_132) {
            var_139 = wp::address(var_geom_solref, var_55, var_40);
            var_142 = wp::load(var_139);
            var_141 = wp::extract(var_142, var_140);
            var_144 = (var_141 > var_143);
            var_132 = var_132 && var_144;
        }
        if (var_132) {
            // solref = mix * geom_solref[solref_id, g1] + (1.0 - mix) * geom_solref[solref_id, g2]       <L 384>
            var_145 = wp::address(var_geom_solref, var_55, var_38);
            var_147 = wp::load(var_145);
            var_146 = wp::mul(var_118, var_147);
            var_149 = wp::sub(var_148, var_118);
            var_150 = wp::address(var_geom_solref, var_55, var_40);
            var_152 = wp::load(var_150);
            var_151 = wp::mul(var_149, var_152);
            var_153 = wp::add(var_146, var_151);
        }
        if (!var_132) {
            // solref = wp.min(geom_solref[solref_id, g1], geom_solref[solref_id, g2])        <L 386>
            var_154 = wp::address(var_geom_solref, var_55, var_38);
            var_155 = wp::address(var_geom_solref, var_55, var_40);
            var_157 = wp::load(var_154);
            var_158 = wp::load(var_155);
            var_156 = wp::min(var_157, var_158);
        }
        var_159 = wp::where(var_132, var_153, var_156);
        // solreffriction = wp.vec2(0.0, 0.0)                                                 <L 388>
        var_162 = wp::vec_t<2, wp::float32>(var_160, var_161);
        // solimp = mix * geom_solimp[solimp_id, g1] + (1.0 - mix) * geom_solimp[solimp_id, g2]       <L 389>
        var_163 = wp::address(var_geom_solimp, var_60, var_38);
        var_165 = wp::load(var_163);
        var_164 = wp::mul(var_118, var_165);
        var_167 = wp::sub(var_166, var_118);
        var_168 = wp::address(var_geom_solimp, var_60, var_40);
        var_170 = wp::load(var_168);
        var_169 = wp::mul(var_167, var_170);
        var_171 = wp::add(var_164, var_169);
    }
    var_172 = wp::where(var_1, var_3, var_119);
    var_173 = wp::where(var_1, var_11, var_131);
    var_174 = wp::where(var_1, var_19, var_159);
    var_175 = wp::where(var_1, var_27, var_162);
    var_176 = wp::where(var_1, var_35, var_171);
    // friction = vec5(                                                                       <L 391>
    // wp.max(MJ_MINMU, friction[0]),                                                         <L 392>
    var_179 = wp::extract(var_173, var_178);
    var_180 = wp::max(var_177, var_179);
    // wp.max(MJ_MINMU, friction[1]),                                                         <L 393>
    var_182 = wp::extract(var_173, var_181);
    var_183 = wp::max(var_177, var_182);
    // wp.max(MJ_MINMU, friction[2]),                                                         <L 394>
    var_185 = wp::extract(var_173, var_184);
    var_186 = wp::max(var_177, var_185);
    // wp.max(MJ_MINMU, friction[3]),                                                         <L 395>
    var_188 = wp::extract(var_173, var_187);
    var_189 = wp::max(var_177, var_188);
    // wp.max(MJ_MINMU, friction[4]),                                                         <L 396>
    var_191 = wp::extract(var_173, var_190);
    var_192 = wp::max(var_177, var_191);
    var_193 = wp::vec_t<5, wp::float32>({var_180, var_183, var_186, var_189, var_192});
    // return condim, friction, solref, solreffriction, solimp                                <L 399>
    ret_0 = var_172;
    ret_1 = var_193;
    ret_2 = var_174;
    ret_3 = var_175;
    ret_4 = var_176;
    return;
}


// /home/cofallen/miniconda3/envs/mjlab/lib/python3.12/site-packages/mujoco_warp/_src/collision_core.py:402
static CUDA_CALLABLE void contact_params_0(
    wp::array_t<wp::int32> var_geom_condim,
    wp::array_t<wp::int32> var_geom_priority,
    wp::array_t<wp::float32> var_geom_solmix,
    wp::array_t<wp::vec_t<2, wp::float32>> var_geom_solref,
    wp::array_t<wp::vec_t<5, wp::float32>> var_geom_solimp,
    wp::array_t<wp::vec_t<3, wp::float32>> var_geom_friction,
    wp::array_t<wp::float32> var_geom_margin,
    wp::array_t<wp::float32> var_geom_gap,
    wp::array_t<wp::int32> var_pair_dim,
    wp::array_t<wp::vec_t<2, wp::float32>> var_pair_solref,
    wp::array_t<wp::vec_t<2, wp::float32>> var_pair_solreffriction,
    wp::array_t<wp::vec_t<5, wp::float32>> var_pair_solimp,
    wp::array_t<wp::float32> var_pair_margin,
    wp::array_t<wp::float32> var_pair_gap,
    wp::array_t<wp::vec_t<5, wp::float32>> var_pair_friction,
    wp::array_t<wp::vec_t<2, wp::int32>> var_collision_pair_in,
    wp::array_t<wp::vec_t<2, wp::int32>> var_collision_pairid_in,
    wp::int32 var_cid,
    wp::int32 var_worldid,
    wp::vec_t<2, wp::int32> & ret_0,
    wp::float32 & ret_1,
    wp::float32 & ret_2,
    wp::int32 & ret_3,
    wp::vec_t<5, wp::float32> & ret_4,
    wp::vec_t<2, wp::float32> & ret_5,
    wp::vec_t<2, wp::float32> & ret_6,
    wp::vec_t<5, wp::float32> & ret_7)
{
    //---------
    // primal vars
    wp::vec_t<2, wp::int32>* var_0;
    wp::vec_t<2, wp::int32> var_1;
    wp::vec_t<2, wp::int32> var_2;
    wp::vec_t<2, wp::int32>* var_3;
    const wp::int32 var_4 = 0;
    wp::int32 var_5;
    wp::vec_t<2, wp::int32> var_6;
    wp::float32 var_7;
    wp::float32 var_8;
    wp::int32 var_9;
    wp::vec_t<5, wp::float32> var_10;
    wp::vec_t<2, wp::float32> var_11;
    wp::vec_t<2, wp::float32> var_12;
    wp::vec_t<5, wp::float32> var_13;
    //---------
    // forward
    // def contact_params(                                                                    <L 403>
    // geoms = collision_pair_in[cid]                                                         <L 431>
    var_0 = wp::address(var_collision_pair_in, var_cid);
    var_2 = wp::load(var_0);
    var_1 = wp::copy(var_2);
    // pairid = collision_pairid_in[cid][0]                                                   <L 432>
    var_3 = wp::address(var_collision_pairid_in, var_cid);
    var_6 = wp::load(var_3);
    var_5 = wp::extract(var_6, var_4);
    // margin, gap = contact_margin_gap(geom_margin, geom_gap, pair_margin, pair_gap, geoms, pairid, worldid)       <L 437>
    contact_margin_gap_0(var_geom_margin, var_geom_gap, var_pair_margin, var_pair_gap, var_1, var_5, var_worldid, var_7, var_8);
    // condim, friction, solref, solreffriction, solimp = contact_material_params(            <L 438>
    // geom_condim,                                                                           <L 439>
    // geom_priority,                                                                         <L 440>
    // geom_solmix,                                                                           <L 441>
    // geom_solref,                                                                           <L 442>
    // geom_solimp,                                                                           <L 443>
    // geom_friction,                                                                         <L 444>
    // pair_dim,                                                                              <L 445>
    // pair_solref,                                                                           <L 446>
    // pair_solreffriction,                                                                   <L 447>
    // pair_solimp,                                                                           <L 448>
    // pair_friction,                                                                         <L 449>
    // geoms,                                                                                 <L 450>
    // pairid,                                                                                <L 451>
    // worldid,                                                                               <L 452>
    contact_material_params_0(var_geom_condim, var_geom_priority, var_geom_solmix, var_geom_solref, var_geom_solimp, var_geom_friction, var_pair_dim, var_pair_solref, var_pair_solreffriction, var_pair_solimp, var_pair_friction, var_1, var_5, var_worldid, var_9, var_10, var_11, var_12, var_13);
    // return geoms, margin, gap, condim, friction, solref, solreffriction, solimp            <L 455>
    ret_0 = var_1;
    ret_1 = var_7;
    ret_2 = var_8;
    ret_3 = var_9;
    ret_4 = var_10;
    ret_5 = var_11;
    ret_6 = var_12;
    ret_7 = var_13;
    return;
}


// /home/cofallen/miniconda3/envs/mjlab/lib/python3.12/site-packages/mujoco_warp/_src/collision_core.py:65
static CUDA_CALLABLE void geom_collision_pair_from_types_0(
    wp::array_t<wp::int32> var_geom_dataid,
    wp::array_t<wp::vec_t<3, wp::float32>> var_geom_size,
    wp::array_t<wp::int32> var_mesh_vertadr,
    wp::array_t<wp::int32> var_mesh_vertnum,
    wp::array_t<wp::int32> var_mesh_graphadr,
    wp::array_t<wp::vec_t<3, wp::float32>> var_mesh_vert,
    wp::array_t<wp::int32> var_mesh_graph,
    wp::array_t<wp::int32> var_mesh_polynum,
    wp::array_t<wp::int32> var_mesh_polyadr,
    wp::array_t<wp::vec_t<3, wp::float32>> var_mesh_polynormal,
    wp::array_t<wp::int32> var_mesh_polyvertadr,
    wp::array_t<wp::int32> var_mesh_polyvertnum,
    wp::array_t<wp::int32> var_mesh_polyvert,
    wp::array_t<wp::int32> var_mesh_polymapadr,
    wp::array_t<wp::int32> var_mesh_polymapnum,
    wp::array_t<wp::int32> var_mesh_polymap,
    wp::array_t<wp::vec_t<3, wp::float32>> var_geom_xpos_in,
    wp::array_t<wp::mat_t<3, 3, wp::float32>> var_geom_xmat_in,
    wp::int32 var_geom_type1,
    wp::int32 var_geom_type2,
    wp::vec_t<2, wp::int32> var_geoms,
    wp::int32 var_worldid,
    Geom_3242f8a8 & ret_0,
    Geom_3242f8a8 & ret_1)
{
    //---------
    // primal vars
    Geom_3242f8a8 var_0;
    Geom_3242f8a8 var_1;
    const wp::int32 var_2 = 0;
    wp::int32 var_3;
    const wp::int32 var_4 = 1;
    wp::int32 var_5;
    wp::vec_t<3, wp::float32>* var_6;
    wp::vec_t<3, wp::float32> var_7;
    wp::vec_t<3, wp::float32> var_8;
    wp::mat_t<3, 3, wp::float32>* var_9;
    wp::mat_t<3, 3, wp::float32> var_10;
    wp::mat_t<3, 3, wp::float32> var_11;
    wp::shape_t* var_12;
    const wp::int32 var_13 = 0;
    wp::int32 var_14;
    wp::shape_t var_15;
    wp::int32 var_16;
    wp::vec_t<3, wp::float32>* var_17;
    wp::vec_t<3, wp::float32> var_18;
    wp::vec_t<3, wp::float32> var_19;
    wp::mat_t<3, 3, wp::float32>* var_20;
    const wp::int32 var_21 = 0;
    const wp::int32 var_22 = 2;
    wp::float32 var_23;
    wp::mat_t<3, 3, wp::float32> var_24;
    wp::mat_t<3, 3, wp::float32>* var_25;
    const wp::int32 var_26 = 1;
    const wp::int32 var_27 = 2;
    wp::float32 var_28;
    wp::mat_t<3, 3, wp::float32> var_29;
    wp::mat_t<3, 3, wp::float32>* var_30;
    const wp::int32 var_31 = 2;
    const wp::int32 var_32 = 2;
    wp::float32 var_33;
    wp::mat_t<3, 3, wp::float32> var_34;
    wp::vec_t<3, wp::float32> var_35;
    wp::vec_t<3, wp::float32>* var_36;
    wp::vec_t<3, wp::float32> var_37;
    wp::vec_t<3, wp::float32> var_38;
    wp::mat_t<3, 3, wp::float32>* var_39;
    wp::mat_t<3, 3, wp::float32> var_40;
    wp::mat_t<3, 3, wp::float32> var_41;
    wp::shape_t* var_42;
    const wp::int32 var_43 = 0;
    wp::int32 var_44;
    wp::shape_t var_45;
    wp::int32 var_46;
    wp::vec_t<3, wp::float32>* var_47;
    wp::vec_t<3, wp::float32> var_48;
    wp::vec_t<3, wp::float32> var_49;
    wp::mat_t<3, 3, wp::float32>* var_50;
    const wp::int32 var_51 = 0;
    const wp::int32 var_52 = 2;
    wp::float32 var_53;
    wp::mat_t<3, 3, wp::float32> var_54;
    wp::mat_t<3, 3, wp::float32>* var_55;
    const wp::int32 var_56 = 1;
    const wp::int32 var_57 = 2;
    wp::float32 var_58;
    wp::mat_t<3, 3, wp::float32> var_59;
    wp::mat_t<3, 3, wp::float32>* var_60;
    const wp::int32 var_61 = 2;
    const wp::int32 var_62 = 2;
    wp::float32 var_63;
    wp::mat_t<3, 3, wp::float32> var_64;
    wp::vec_t<3, wp::float32> var_65;
    wp::shape_t* var_66;
    const wp::int32 var_67 = 0;
    wp::int32 var_68;
    wp::shape_t var_69;
    wp::int32 var_70;
    const wp::int32 var_71 = 7;
    bool var_72;
    wp::int32* var_73;
    wp::int32 var_74;
    wp::int32 var_75;
    const wp::int32 var_76 = 0;
    bool var_77;
    wp::int32* var_78;
    const wp::int32 var_79 = -1;
    wp::int32 var_80;
    wp::int32 var_81;
    const wp::int32 var_82 = 0;
    bool var_83;
    wp::int32* var_84;
    const wp::int32 var_85 = -1;
    wp::int32 var_86;
    wp::int32 var_87;
    const wp::int32 var_88 = 0;
    bool var_89;
    wp::int32* var_90;
    const wp::int32 var_91 = -1;
    wp::int32 var_92;
    wp::int32 var_93;
    const wp::int32 var_94 = 0;
    bool var_95;
    wp::int32* var_96;
    const wp::int32 var_97 = -1;
    wp::int32 var_98;
    wp::int32 var_99;
    const wp::int32 var_100 = 0;
    bool var_101;
    wp::int32* var_102;
    const wp::int32 var_103 = -1;
    wp::int32 var_104;
    wp::int32 var_105;
    const wp::int32 var_106 = 7;
    bool var_107;
    wp::int32* var_108;
    wp::int32 var_109;
    wp::int32 var_110;
    const wp::int32 var_111 = 0;
    bool var_112;
    wp::int32* var_113;
    const wp::int32 var_114 = -1;
    wp::int32 var_115;
    wp::int32 var_116;
    const wp::int32 var_117 = 0;
    bool var_118;
    wp::int32* var_119;
    const wp::int32 var_120 = -1;
    wp::int32 var_121;
    wp::int32 var_122;
    const wp::int32 var_123 = 0;
    bool var_124;
    wp::int32* var_125;
    const wp::int32 var_126 = -1;
    wp::int32 var_127;
    wp::int32 var_128;
    const wp::int32 var_129 = 0;
    bool var_130;
    wp::int32* var_131;
    const wp::int32 var_132 = -1;
    wp::int32 var_133;
    wp::int32 var_134;
    const wp::int32 var_135 = 0;
    bool var_136;
    wp::int32* var_137;
    const wp::int32 var_138 = -1;
    wp::int32 var_139;
    wp::int32 var_140;
    wp::int32 var_141;
    const wp::int32 var_142 = -1;
    const wp::float32 var_143 = 0.0;
    const wp::int32 var_144 = -1;
    const wp::float32 var_145 = 0.0;
    //---------
    // forward
    // def geom_collision_pair_from_types(                                                    <L 66>
    // geom1 = Geom()                                                                         <L 93>
    var_0 = Geom_3242f8a8();
    // geom2 = Geom()                                                                         <L 94>
    var_1 = Geom_3242f8a8();
    // g1 = geoms[0]                                                                          <L 96>
    var_3 = wp::extract(var_geoms, var_2);
    // g2 = geoms[1]                                                                          <L 97>
    var_5 = wp::extract(var_geoms, var_4);
    // geom1.pos = geom_xpos_in[worldid, g1]                                                  <L 99>
    var_6 = wp::address(var_geom_xpos_in, var_worldid, var_3);
    var_8 = wp::load(var_6);
    var_7 = wp::copy(var_8);
    var_0.pos = var_7;
    // geom1.rot = geom_xmat_in[worldid, g1]                                                  <L 100>
    var_9 = wp::address(var_geom_xmat_in, var_worldid, var_3);
    var_11 = wp::load(var_9);
    var_10 = wp::copy(var_11);
    var_0.rot = var_10;
    // geom1.size = geom_size[worldid % geom_size.shape[0], g1]                               <L 101>
    var_12 = &(var_geom_size.shape);
    var_15 = wp::load(var_12);
    var_14 = wp::extract(var_15, var_13);
    var_16 = wp::mod(var_worldid, var_14);
    var_17 = wp::address(var_geom_size, var_16, var_3);
    var_19 = wp::load(var_17);
    var_18 = wp::copy(var_19);
    var_0.size = var_18;
    // geom1.normal = wp.vec3(geom1.rot[0, 2], geom1.rot[1, 2], geom1.rot[2, 2])              <L 103>
    var_20 = &((var_0).rot);
    var_24 = wp::load(var_20);
    var_23 = wp::extract(var_24, var_21, var_22);
    var_25 = &((var_0).rot);
    var_29 = wp::load(var_25);
    var_28 = wp::extract(var_29, var_26, var_27);
    var_30 = &((var_0).rot);
    var_34 = wp::load(var_30);
    var_33 = wp::extract(var_34, var_31, var_32);
    var_35 = wp::vec_t<3, wp::float32>(var_23, var_28, var_33);
    var_0.normal = var_35;
    // geom2.pos = geom_xpos_in[worldid, g2]                                                  <L 105>
    var_36 = wp::address(var_geom_xpos_in, var_worldid, var_5);
    var_38 = wp::load(var_36);
    var_37 = wp::copy(var_38);
    var_1.pos = var_37;
    // geom2.rot = geom_xmat_in[worldid, g2]                                                  <L 106>
    var_39 = wp::address(var_geom_xmat_in, var_worldid, var_5);
    var_41 = wp::load(var_39);
    var_40 = wp::copy(var_41);
    var_1.rot = var_40;
    // geom2.size = geom_size[worldid % geom_size.shape[0], g2]                               <L 107>
    var_42 = &(var_geom_size.shape);
    var_45 = wp::load(var_42);
    var_44 = wp::extract(var_45, var_43);
    var_46 = wp::mod(var_worldid, var_44);
    var_47 = wp::address(var_geom_size, var_46, var_5);
    var_49 = wp::load(var_47);
    var_48 = wp::copy(var_49);
    var_1.size = var_48;
    // geom2.normal = wp.vec3(geom2.rot[0, 2], geom2.rot[1, 2], geom2.rot[2, 2])              <L 109>
    var_50 = &((var_1).rot);
    var_54 = wp::load(var_50);
    var_53 = wp::extract(var_54, var_51, var_52);
    var_55 = &((var_1).rot);
    var_59 = wp::load(var_55);
    var_58 = wp::extract(var_59, var_56, var_57);
    var_60 = &((var_1).rot);
    var_64 = wp::load(var_60);
    var_63 = wp::extract(var_64, var_61, var_62);
    var_65 = wp::vec_t<3, wp::float32>(var_53, var_58, var_63);
    var_1.normal = var_65;
    // dataid_setid = worldid % geom_dataid.shape[0]                                          <L 111>
    var_66 = &(var_geom_dataid.shape);
    var_69 = wp::load(var_66);
    var_68 = wp::extract(var_69, var_67);
    var_70 = wp::mod(var_worldid, var_68);
    // if geom_type1 == GeomType.MESH:                                                        <L 113>
    var_72 = (var_geom_type1 == var_71);
    if (var_72) {
        // dataid = geom_dataid[dataid_setid, g1]                                             <L 114>
        var_73 = wp::address(var_geom_dataid, var_70, var_3);
        var_75 = wp::load(var_73);
        var_74 = wp::copy(var_75);
        // geom1.vertadr = wp.where(dataid >= 0, mesh_vertadr[dataid], -1)                    <L 115>
        var_77 = (var_74 >= var_76);
        var_78 = wp::address(var_mesh_vertadr, var_74);
        var_81 = wp::load(var_78);
        var_80 = wp::where(var_77, var_81, var_79);
        var_0.vertadr = var_80;
        // geom1.vertnum = wp.where(dataid >= 0, mesh_vertnum[dataid], -1)                    <L 116>
        var_83 = (var_74 >= var_82);
        var_84 = wp::address(var_mesh_vertnum, var_74);
        var_87 = wp::load(var_84);
        var_86 = wp::where(var_83, var_87, var_85);
        var_0.vertnum = var_86;
        // geom1.graphadr = wp.where(dataid >= 0, mesh_graphadr[dataid], -1)                  <L 117>
        var_89 = (var_74 >= var_88);
        var_90 = wp::address(var_mesh_graphadr, var_74);
        var_93 = wp::load(var_90);
        var_92 = wp::where(var_89, var_93, var_91);
        var_0.graphadr = var_92;
        // geom1.mesh_polynum = wp.where(dataid >= 0, mesh_polynum[dataid], -1)               <L 118>
        var_95 = (var_74 >= var_94);
        var_96 = wp::address(var_mesh_polynum, var_74);
        var_99 = wp::load(var_96);
        var_98 = wp::where(var_95, var_99, var_97);
        var_0.mesh_polynum = var_98;
        // geom1.mesh_polyadr = wp.where(dataid >= 0, mesh_polyadr[dataid], -1)               <L 119>
        var_101 = (var_74 >= var_100);
        var_102 = wp::address(var_mesh_polyadr, var_74);
        var_105 = wp::load(var_102);
        var_104 = wp::where(var_101, var_105, var_103);
        var_0.mesh_polyadr = var_104;
        // geom1.vert = mesh_vert                                                             <L 121>
        var_0.vert = var_mesh_vert;
        // geom1.graph = mesh_graph                                                           <L 122>
        var_0.graph = var_mesh_graph;
        // geom1.mesh_polynormal = mesh_polynormal                                            <L 123>
        var_0.mesh_polynormal = var_mesh_polynormal;
        // geom1.mesh_polyvertadr = mesh_polyvertadr                                          <L 124>
        var_0.mesh_polyvertadr = var_mesh_polyvertadr;
        // geom1.mesh_polyvertnum = mesh_polyvertnum                                          <L 125>
        var_0.mesh_polyvertnum = var_mesh_polyvertnum;
        // geom1.mesh_polyvert = mesh_polyvert                                                <L 126>
        var_0.mesh_polyvert = var_mesh_polyvert;
        // geom1.mesh_polymapadr = mesh_polymapadr                                            <L 127>
        var_0.mesh_polymapadr = var_mesh_polymapadr;
        // geom1.mesh_polymapnum = mesh_polymapnum                                            <L 128>
        var_0.mesh_polymapnum = var_mesh_polymapnum;
        // geom1.mesh_polymap = mesh_polymap                                                  <L 129>
        var_0.mesh_polymap = var_mesh_polymap;
    }
    // if geom_type2 == GeomType.MESH:                                                        <L 131>
    var_107 = (var_geom_type2 == var_106);
    if (var_107) {
        // dataid = geom_dataid[dataid_setid, g2]                                             <L 132>
        var_108 = wp::address(var_geom_dataid, var_70, var_5);
        var_110 = wp::load(var_108);
        var_109 = wp::copy(var_110);
        // geom2.vertadr = wp.where(dataid >= 0, mesh_vertadr[dataid], -1)                    <L 133>
        var_112 = (var_109 >= var_111);
        var_113 = wp::address(var_mesh_vertadr, var_109);
        var_116 = wp::load(var_113);
        var_115 = wp::where(var_112, var_116, var_114);
        var_1.vertadr = var_115;
        // geom2.vertnum = wp.where(dataid >= 0, mesh_vertnum[dataid], -1)                    <L 134>
        var_118 = (var_109 >= var_117);
        var_119 = wp::address(var_mesh_vertnum, var_109);
        var_122 = wp::load(var_119);
        var_121 = wp::where(var_118, var_122, var_120);
        var_1.vertnum = var_121;
        // geom2.graphadr = wp.where(dataid >= 0, mesh_graphadr[dataid], -1)                  <L 135>
        var_124 = (var_109 >= var_123);
        var_125 = wp::address(var_mesh_graphadr, var_109);
        var_128 = wp::load(var_125);
        var_127 = wp::where(var_124, var_128, var_126);
        var_1.graphadr = var_127;
        // geom2.mesh_polynum = wp.where(dataid >= 0, mesh_polynum[dataid], -1)               <L 136>
        var_130 = (var_109 >= var_129);
        var_131 = wp::address(var_mesh_polynum, var_109);
        var_134 = wp::load(var_131);
        var_133 = wp::where(var_130, var_134, var_132);
        var_1.mesh_polynum = var_133;
        // geom2.mesh_polyadr = wp.where(dataid >= 0, mesh_polyadr[dataid], -1)               <L 137>
        var_136 = (var_109 >= var_135);
        var_137 = wp::address(var_mesh_polyadr, var_109);
        var_140 = wp::load(var_137);
        var_139 = wp::where(var_136, var_140, var_138);
        var_1.mesh_polyadr = var_139;
        // geom2.vert = mesh_vert                                                             <L 139>
        var_1.vert = var_mesh_vert;
        // geom2.graph = mesh_graph                                                           <L 140>
        var_1.graph = var_mesh_graph;
        // geom2.mesh_polynormal = mesh_polynormal                                            <L 141>
        var_1.mesh_polynormal = var_mesh_polynormal;
        // geom2.mesh_polyvertadr = mesh_polyvertadr                                          <L 142>
        var_1.mesh_polyvertadr = var_mesh_polyvertadr;
        // geom2.mesh_polyvertnum = mesh_polyvertnum                                          <L 143>
        var_1.mesh_polyvertnum = var_mesh_polyvertnum;
        // geom2.mesh_polyvert = mesh_polyvert                                                <L 144>
        var_1.mesh_polyvert = var_mesh_polyvert;
        // geom2.mesh_polymapadr = mesh_polymapadr                                            <L 145>
        var_1.mesh_polymapadr = var_mesh_polymapadr;
        // geom2.mesh_polymapnum = mesh_polymapnum                                            <L 146>
        var_1.mesh_polymapnum = var_mesh_polymapnum;
        // geom2.mesh_polymap = mesh_polymap                                                  <L 147>
        var_1.mesh_polymap = var_mesh_polymap;
    }
    var_141 = wp::where(var_107, var_109, var_74);
    // geom1.index = -1                                                                       <L 149>
    var_0.index = var_142;
    // geom1.margin = 0.0                                                                     <L 150>
    var_0.margin = var_143;
    // geom2.index = -1                                                                       <L 152>
    var_1.index = var_144;
    // geom2.margin = 0.0                                                                     <L 153>
    var_1.margin = var_145;
    // return geom1, geom2                                                                    <L 155>
    ret_0 = var_0;
    ret_1 = var_1;
    return;
}


// /home/cofallen/miniconda3/envs/mjlab/lib/python3.12/site-packages/mujoco_warp/_src/collision_core.py:158
static CUDA_CALLABLE void geom_collision_pair_0(
    wp::array_t<wp::int32> var_geom_type,
    wp::array_t<wp::int32> var_geom_dataid,
    wp::array_t<wp::vec_t<3, wp::float32>> var_geom_size,
    wp::array_t<wp::int32> var_mesh_vertadr,
    wp::array_t<wp::int32> var_mesh_vertnum,
    wp::array_t<wp::int32> var_mesh_graphadr,
    wp::array_t<wp::vec_t<3, wp::float32>> var_mesh_vert,
    wp::array_t<wp::int32> var_mesh_graph,
    wp::array_t<wp::int32> var_mesh_polynum,
    wp::array_t<wp::int32> var_mesh_polyadr,
    wp::array_t<wp::vec_t<3, wp::float32>> var_mesh_polynormal,
    wp::array_t<wp::int32> var_mesh_polyvertadr,
    wp::array_t<wp::int32> var_mesh_polyvertnum,
    wp::array_t<wp::int32> var_mesh_polyvert,
    wp::array_t<wp::int32> var_mesh_polymapadr,
    wp::array_t<wp::int32> var_mesh_polymapnum,
    wp::array_t<wp::int32> var_mesh_polymap,
    wp::array_t<wp::vec_t<3, wp::float32>> var_geom_xpos_in,
    wp::array_t<wp::mat_t<3, 3, wp::float32>> var_geom_xmat_in,
    wp::vec_t<2, wp::int32> var_geoms,
    wp::int32 var_worldid,
    Geom_3242f8a8 & ret_0,
    Geom_3242f8a8 & ret_1)
{
    //---------
    // primal vars
    const wp::int32 var_0 = 0;
    wp::int32 var_1;
    wp::int32* var_2;
    wp::int32 var_3;
    wp::int32 var_4;
    const wp::int32 var_5 = 1;
    wp::int32 var_6;
    wp::int32* var_7;
    wp::int32 var_8;
    wp::int32 var_9;
    Geom_3242f8a8 var_10;
    Geom_3242f8a8 var_11;
    //---------
    // forward
    // def geom_collision_pair(                                                               <L 159>
    // geom_type1 = geom_type[geoms[0]]                                                       <L 185>
    var_1 = wp::extract(var_geoms, var_0);
    var_2 = wp::address(var_geom_type, var_1);
    var_4 = wp::load(var_2);
    var_3 = wp::copy(var_4);
    // geom_type2 = geom_type[geoms[1]]                                                       <L 186>
    var_6 = wp::extract(var_geoms, var_5);
    var_7 = wp::address(var_geom_type, var_6);
    var_9 = wp::load(var_7);
    var_8 = wp::copy(var_9);
    // return geom_collision_pair_from_types(                                                 <L 187>
    // geom_dataid,                                                                           <L 188>
    // geom_size,                                                                             <L 189>
    // mesh_vertadr,                                                                          <L 190>
    // mesh_vertnum,                                                                          <L 191>
    // mesh_graphadr,                                                                         <L 192>
    // mesh_vert,                                                                             <L 193>
    // mesh_graph,                                                                            <L 194>
    // mesh_polynum,                                                                          <L 195>
    // mesh_polyadr,                                                                          <L 196>
    // mesh_polynormal,                                                                       <L 197>
    // mesh_polyvertadr,                                                                      <L 198>
    // mesh_polyvertnum,                                                                      <L 199>
    // mesh_polyvert,                                                                         <L 200>
    // mesh_polymapadr,                                                                       <L 201>
    // mesh_polymapnum,                                                                       <L 202>
    // mesh_polymap,                                                                          <L 203>
    // geom_xpos_in,                                                                          <L 204>
    // geom_xmat_in,                                                                          <L 205>
    // geom_type1,                                                                            <L 206>
    // geom_type2,                                                                            <L 207>
    // geoms,                                                                                 <L 208>
    // worldid,                                                                               <L 209>
    geom_collision_pair_from_types_0(var_geom_dataid, var_geom_size, var_mesh_vertadr, var_mesh_vertnum, var_mesh_graphadr, var_mesh_vert, var_mesh_graph, var_mesh_polynum, var_mesh_polyadr, var_mesh_polynormal, var_mesh_polyvertadr, var_mesh_polyvertnum, var_mesh_polyvert, var_mesh_polymapadr, var_mesh_polymapnum, var_mesh_polymap, var_geom_xpos_in, var_geom_xmat_in, var_3, var_8, var_geoms, var_worldid, var_10, var_11);
    ret_0 = var_10;
    ret_1 = var_11;
    return;
}


// /home/cofallen/miniconda3/envs/mjlab/lib/python3.12/site-packages/mujoco_warp/_src/collision_primitive_core.py:459
static CUDA_CALLABLE void plane_cylinder_0(
    wp::vec_t<3, wp::float32> var_plane_normal,
    wp::vec_t<3, wp::float32> var_plane_pos,
    wp::vec_t<3, wp::float32> var_cylinder_center,
    wp::vec_t<3, wp::float32> var_cylinder_axis,
    wp::float32 var_cylinder_radius,
    wp::float32 var_cylinder_half_height,
    wp::vec_t<4, wp::float32> & ret_0,
    wp::mat_t<4, 3, wp::float32> & ret_1,
    wp::vec_t<3, wp::float32> & ret_2)
{
    //---------
    // primal vars
    const wp::float32 var_0 = 10000000000.0;
    wp::vec_t<4, wp::float32> var_1;
    wp::mat_t<4, 3, wp::float32> var_2;
    const wp::int32 var_3 = 0;
    wp::vec_t<3, wp::float32> var_4;
    wp::vec_t<3, wp::float32> var_5;
    wp::float32 var_6;
    const wp::int32 var_7 = 0;
    bool var_8;
    wp::vec_t<3, wp::float32> var_9;
    wp::float32 var_10;
    wp::vec_t<3, wp::float32> var_11;
    wp::float32 var_12;
    wp::vec_t<3, wp::float32> var_13;
    wp::float32 var_14;
    wp::vec_t<3, wp::float32> var_15;
    wp::vec_t<3, wp::float32> var_16;
    wp::float32 var_17;
    const wp::float32 var_18 = 1e-12;
    bool var_19;
    wp::float32 var_20;
    wp::float32 var_21;
    wp::vec_t<3, wp::float32> var_22;
    const wp::float32 var_23 = 1.0;
    const wp::float32 var_24 = 0.0;
    const wp::float32 var_25 = 0.0;
    wp::vec_t<3, wp::float32> var_26;
    wp::vec_t<3, wp::float32> var_27;
    wp::vec_t<3, wp::float32> var_28;
    wp::float32 var_29;
    wp::vec_t<3, wp::float32> var_30;
    wp::float32 var_31;
    wp::float32 var_32;
    wp::float32 var_33;
    wp::vec_t<3, wp::float32> var_34;
    wp::vec_t<3, wp::float32> var_35;
    const wp::float32 var_36 = 0.5;
    wp::float32 var_37;
    wp::vec_t<3, wp::float32> var_38;
    wp::vec_t<3, wp::float32> var_39;
    const wp::int32 var_40 = 1;
    wp::int32 var_41;
    wp::float32 var_42;
    wp::float32 var_43;
    wp::vec_t<3, wp::float32> var_44;
    wp::vec_t<3, wp::float32> var_45;
    const wp::float32 var_46 = 0.5;
    wp::float32 var_47;
    wp::vec_t<3, wp::float32> var_48;
    wp::vec_t<3, wp::float32> var_49;
    const wp::int32 var_50 = 1;
    wp::int32 var_51;
    wp::float32 var_52;
    const wp::float32 var_53 = 0.5;
    wp::float32 var_54;
    wp::float32 var_55;
    wp::float32 var_56;
    wp::vec_t<3, wp::float32> var_57;
    wp::vec_t<3, wp::float32> var_58;
    const wp::float32 var_59 = 3.0;
    wp::float32 var_60;
    wp::float32 var_61;
    const wp::float32 var_62 = 0.5;
    wp::float32 var_63;
    wp::vec_t<3, wp::float32> var_64;
    wp::vec_t<3, wp::float32> var_65;
    wp::vec_t<3, wp::float32> var_66;
    const wp::float32 var_67 = 0.5;
    wp::vec_t<3, wp::float32> var_68;
    wp::vec_t<3, wp::float32> var_69;
    const wp::float32 var_70 = 0.5;
    wp::float32 var_71;
    wp::vec_t<3, wp::float32> var_72;
    wp::vec_t<3, wp::float32> var_73;
    const wp::int32 var_74 = 1;
    wp::int32 var_75;
    wp::vec_t<3, wp::float32> var_76;
    wp::vec_t<3, wp::float32> var_77;
    const wp::float32 var_78 = 0.5;
    wp::vec_t<3, wp::float32> var_79;
    wp::vec_t<3, wp::float32> var_80;
    const wp::float32 var_81 = 0.5;
    wp::float32 var_82;
    wp::vec_t<3, wp::float32> var_83;
    wp::vec_t<3, wp::float32> var_84;
    const wp::int32 var_85 = 1;
    wp::int32 var_86;
    //---------
    // forward
    // def plane_cylinder(                                                                    <L 460>
    // contact_dist = wp.vec4(MJ_MAXVAL)                                                      <L 485>
    var_1 = wp::vec_t<4, wp::float32>(var_0);
    // contact_pos = mat43f()                                                                 <L 486>
    var_2 = wp::mat_t<4, 3, wp::float32>();
    // contact_count = 0                                                                      <L 487>
    // n = plane_normal                                                                       <L 489>
    var_4 = wp::copy(var_plane_normal);
    // axis = cylinder_axis                                                                   <L 490>
    var_5 = wp::copy(var_cylinder_axis);
    // prjaxis = wp.dot(n, axis)                                                              <L 493>
    var_6 = wp::dot(var_4, var_5);
    // if prjaxis > 0:                                                                        <L 494>
    var_8 = (var_6 > var_7);
    if (var_8) {
        // axis = -axis                                                                       <L 495>
        var_9 = wp::neg(var_5);
        // prjaxis = -prjaxis                                                                 <L 496>
        var_10 = wp::neg(var_6);
    }
    var_11 = wp::where(var_8, var_9, var_5);
    var_12 = wp::where(var_8, var_10, var_6);
    // dist0 = wp.dot(cylinder_center - plane_pos, n)                                         <L 499>
    var_13 = wp::sub(var_cylinder_center, var_plane_pos);
    var_14 = wp::dot(var_13, var_4);
    // vec = axis * prjaxis - n                                                               <L 502>
    var_15 = wp::mul(var_11, var_12);
    var_16 = wp::sub(var_15, var_4);
    // len_sqr = wp.dot(vec, vec)                                                             <L 503>
    var_17 = wp::dot(var_16, var_16);
    // vec = wp.where(                                                                        <L 507>
    // len_sqr >= 1e-12,                                                                      <L 508>
    var_19 = (var_17 >= var_18);
    // vec * safe_div(cylinder_radius, wp.sqrt(len_sqr)),                                     <L 509>
    var_20 = wp::sqrt(var_17);
    var_21 = safe_div_0(var_cylinder_radius, var_20);
    var_22 = wp::mul(var_16, var_21);
    // wp.vec3(1.0, 0.0, 0.0) * cylinder_radius,  # Default x-axis when degenerate            <L 510>
    var_26 = wp::vec_t<3, wp::float32>(var_23, var_24, var_25);
    var_27 = wp::mul(var_26, var_cylinder_radius);
    var_28 = wp::where(var_19, var_22, var_27);
    // prjvec = wp.dot(vec, n)                                                                <L 514>
    var_29 = wp::dot(var_28, var_4);
    // axis = axis * cylinder_half_height                                                     <L 517>
    var_30 = wp::mul(var_11, var_cylinder_half_height);
    // prjaxis = prjaxis * cylinder_half_height                                               <L 518>
    var_31 = wp::mul(var_12, var_cylinder_half_height);
    // dist1 = dist0 + prjaxis + prjvec                                                       <L 521>
    var_32 = wp::add(var_14, var_31);
    var_33 = wp::add(var_32, var_29);
    // pos1 = cylinder_center + vec + axis - n * (dist1 * 0.5)                                <L 522>
    var_34 = wp::add(var_cylinder_center, var_28);
    var_35 = wp::add(var_34, var_30);
    var_37 = wp::mul(var_33, var_36);
    var_38 = wp::mul(var_4, var_37);
    var_39 = wp::sub(var_35, var_38);
    // contact_dist[contact_count] = dist1                                                    <L 523>
    wp::assign_inplace(var_1, var_3, var_33);
    // contact_pos[contact_count] = pos1                                                      <L 524>
    wp::assign_inplace(var_2, var_3, var_39);
    // contact_count = contact_count + 1                                                      <L 525>
    var_41 = wp::add(var_3, var_40);
    // dist2 = dist0 - prjaxis + prjvec                                                       <L 528>
    var_42 = wp::sub(var_14, var_31);
    var_43 = wp::add(var_42, var_29);
    // pos2 = cylinder_center + vec - axis - n * (dist2 * 0.5)                                <L 529>
    var_44 = wp::add(var_cylinder_center, var_28);
    var_45 = wp::sub(var_44, var_30);
    var_47 = wp::mul(var_43, var_46);
    var_48 = wp::mul(var_4, var_47);
    var_49 = wp::sub(var_45, var_48);
    // contact_dist[contact_count] = dist2                                                    <L 530>
    wp::assign_inplace(var_1, var_41, var_43);
    // contact_pos[contact_count] = pos2                                                      <L 531>
    wp::assign_inplace(var_2, var_41, var_49);
    // contact_count = contact_count + 1                                                      <L 532>
    var_51 = wp::add(var_41, var_50);
    // prjvec1 = -prjvec * 0.5                                                                <L 535>
    var_52 = wp::neg(var_29);
    var_54 = wp::mul(var_52, var_53);
    // dist3 = dist0 + prjaxis + prjvec1                                                      <L 536>
    var_55 = wp::add(var_14, var_31);
    var_56 = wp::add(var_55, var_54);
    // vec1 = wp.cross(vec, axis)                                                             <L 538>
    var_57 = wp::cross(var_28, var_30);
    // vec1 = wp.normalize(vec1) * (cylinder_radius * wp.sqrt(3.0) * 0.5)                     <L 539>
    var_58 = wp::normalize(var_57);
    var_60 = wp::sqrt(var_59);
    var_61 = wp::mul(var_cylinder_radius, var_60);
    var_63 = wp::mul(var_61, var_62);
    var_64 = wp::mul(var_58, var_63);
    // pos3 = cylinder_center + vec1 + axis - vec * 0.5 - n * (dist3 * 0.5)                   <L 542>
    var_65 = wp::add(var_cylinder_center, var_64);
    var_66 = wp::add(var_65, var_30);
    var_68 = wp::mul(var_28, var_67);
    var_69 = wp::sub(var_66, var_68);
    var_71 = wp::mul(var_56, var_70);
    var_72 = wp::mul(var_4, var_71);
    var_73 = wp::sub(var_69, var_72);
    // contact_dist[contact_count] = dist3                                                    <L 543>
    wp::assign_inplace(var_1, var_51, var_56);
    // contact_pos[contact_count] = pos3                                                      <L 544>
    wp::assign_inplace(var_2, var_51, var_73);
    // contact_count = contact_count + 1                                                      <L 545>
    var_75 = wp::add(var_51, var_74);
    // pos4 = cylinder_center - vec1 + axis - vec * 0.5 - n * (dist3 * 0.5)                   <L 548>
    var_76 = wp::sub(var_cylinder_center, var_64);
    var_77 = wp::add(var_76, var_30);
    var_79 = wp::mul(var_28, var_78);
    var_80 = wp::sub(var_77, var_79);
    var_82 = wp::mul(var_56, var_81);
    var_83 = wp::mul(var_4, var_82);
    var_84 = wp::sub(var_80, var_83);
    // contact_dist[contact_count] = dist3                                                    <L 549>
    wp::assign_inplace(var_1, var_75, var_56);
    // contact_pos[contact_count] = pos4                                                      <L 550>
    wp::assign_inplace(var_2, var_75, var_84);
    // contact_count = contact_count + 1                                                      <L 551>
    var_86 = wp::add(var_75, var_85);
    // return contact_dist, contact_pos, n                                                    <L 553>
    ret_0 = var_1;
    ret_1 = var_2;
    ret_2 = var_4;
    return;
}


// /home/cofallen/miniconda3/envs/mjlab/lib/python3.12/site-packages/mujoco_warp/_src/math.py:202
static CUDA_CALLABLE void orthogonals_0(
    wp::vec_t<3, wp::float32> var_a,
    wp::vec_t<3, wp::float32> & ret_0,
    wp::vec_t<3, wp::float32> & ret_1)
{
    //---------
    // primal vars
    const wp::float32 var_0 = 0.0;
    const wp::float32 var_1 = 1.0;
    const wp::float32 var_2 = 0.0;
    wp::vec_t<3, wp::float32> var_3;
    const wp::float32 var_4 = 0.0;
    const wp::float32 var_5 = 0.0;
    const wp::float32 var_6 = 1.0;
    wp::vec_t<3, wp::float32> var_7;
    bool var_8;
    const wp::float32 var_9 = -0.5;
    const wp::int32 var_10 = 1;
    wp::float32 var_11;
    bool var_12;
    const wp::int32 var_13 = 1;
    wp::float32 var_14;
    const wp::float32 var_15 = 0.5;
    bool var_16;
    wp::vec_t<3, wp::float32> var_17;
    wp::float32 var_18;
    wp::vec_t<3, wp::float32> var_19;
    wp::vec_t<3, wp::float32> var_20;
    wp::vec_t<3, wp::float32> var_21;
    wp::float32 var_22;
    const wp::float32 var_23 = 0.0;
    bool var_24;
    const wp::float32 var_25 = 0.0;
    const wp::float32 var_26 = 0.0;
    const wp::float32 var_27 = 0.0;
    wp::vec_t<3, wp::float32> var_28;
    wp::vec_t<3, wp::float32> var_29;
    wp::vec_t<3, wp::float32> var_30;
    //---------
    // forward
    // def orthogonals(a: wp.vec3):                                                           <L 203>
    // y = wp.vec3(0.0, 1.0, 0.0)                                                             <L 204>
    var_3 = wp::vec_t<3, wp::float32>(var_0, var_1, var_2);
    // z = wp.vec3(0.0, 0.0, 1.0)                                                             <L 205>
    var_7 = wp::vec_t<3, wp::float32>(var_4, var_5, var_6);
    // b = wp.where((-0.5 < a[1]) and (a[1] < 0.5), y, z)                                     <L 206>
    var_11 = wp::extract(var_a, var_10);
    var_12 = (var_9 < var_11);
    var_8 = var_12;
    if (var_8) {
        var_14 = wp::extract(var_a, var_13);
        var_16 = (var_14 < var_15);
        var_8 = var_8 && var_16;
    }
    var_17 = wp::where(var_8, var_3, var_7);
    // b = b - a * wp.dot(a, b)                                                               <L 207>
    var_18 = wp::dot(var_a, var_17);
    var_19 = wp::mul(var_a, var_18);
    var_20 = wp::sub(var_17, var_19);
    // b = wp.normalize(b)                                                                    <L 208>
    var_21 = wp::normalize(var_20);
    // if wp.length(a) == 0.0:                                                                <L 209>
    var_22 = wp::length(var_a);
    var_24 = (var_22 == var_23);
    if (var_24) {
        // b = wp.vec3(0.0, 0.0, 0.0)                                                         <L 210>
        var_28 = wp::vec_t<3, wp::float32>(var_25, var_26, var_27);
    }
    var_29 = wp::where(var_24, var_28, var_21);
    // c = wp.cross(a, b)                                                                     <L 211>
    var_30 = wp::cross(var_a, var_29);
    // return b, c                                                                            <L 213>
    ret_0 = var_29;
    ret_1 = var_30;
    return;
}


// /home/cofallen/miniconda3/envs/mjlab/lib/python3.12/site-packages/mujoco_warp/_src/math.py:246
static CUDA_CALLABLE wp::mat_t<3, 3, wp::float32> make_frame_0(
    wp::vec_t<3, wp::float32> var_a)
{
    //---------
    // primal vars
    wp::vec_t<3, wp::float32> var_0;
    wp::vec_t<3, wp::float32> var_1;
    wp::vec_t<3, wp::float32> var_2;
    const wp::int32 var_3 = 0;
    wp::float32 var_4;
    const wp::int32 var_5 = 1;
    wp::float32 var_6;
    const wp::int32 var_7 = 2;
    wp::float32 var_8;
    const wp::int32 var_9 = 0;
    wp::float32 var_10;
    const wp::int32 var_11 = 1;
    wp::float32 var_12;
    const wp::int32 var_13 = 2;
    wp::float32 var_14;
    const wp::int32 var_15 = 0;
    wp::float32 var_16;
    const wp::int32 var_17 = 1;
    wp::float32 var_18;
    const wp::int32 var_19 = 2;
    wp::float32 var_20;
    wp::mat_t<3, 3, wp::float32> var_21;
    //---------
    // forward
    // def make_frame(a: wp.vec3):                                                            <L 247>
    // a = wp.normalize(a)                                                                    <L 248>
    var_0 = wp::normalize(var_a);
    // b, c = orthogonals(a)                                                                  <L 249>
    orthogonals_0(var_0, var_1, var_2);
    // return wp.mat33(                                                                       <L 252>
    // a.x, a.y, a.z,                                                                         <L 253>
    var_4 = wp::extract(var_0, var_3);
    var_6 = wp::extract(var_0, var_5);
    var_8 = wp::extract(var_0, var_7);
    // b.x, b.y, b.z,                                                                         <L 254>
    var_10 = wp::extract(var_1, var_9);
    var_12 = wp::extract(var_1, var_11);
    var_14 = wp::extract(var_1, var_13);
    // c.x, c.y, c.z                                                                          <L 255>
    var_16 = wp::extract(var_2, var_15);
    var_18 = wp::extract(var_2, var_17);
    var_20 = wp::extract(var_2, var_19);
    var_21 = wp::mat_t<3, 3, wp::float32>(var_4, var_6, var_8, var_10, var_12, var_14, var_16, var_18, var_20);
    return var_21;
}


// /home/cofallen/miniconda3/envs/mjlab/lib/python3.12/site-packages/mujoco_warp/_src/collision_core.py:213
static CUDA_CALLABLE wp::int32 write_contact_0(
    wp::int32 var_naconmax_in,
    wp::int32 var_id_,
    wp::float32 var_dist_in,
    wp::vec_t<3, wp::float32> var_pos_in,
    wp::mat_t<3, 3, wp::float32> var_frame_in,
    wp::float32 var_margin_in,
    wp::float32 var_gap_in,
    wp::int32 var_condim_in,
    wp::vec_t<5, wp::float32> var_friction_in,
    wp::vec_t<2, wp::float32> var_solref_in,
    wp::vec_t<2, wp::float32> var_solreffriction_in,
    wp::vec_t<5, wp::float32> var_solimp_in,
    wp::vec_t<2, wp::int32> var_geoms_in,
    wp::vec_t<2, wp::int32> var_pairid_in,
    wp::int32 var_worldid_in,
    wp::array_t<wp::float32> var_contact_dist_out,
    wp::array_t<wp::vec_t<3, wp::float32>> var_contact_pos_out,
    wp::array_t<wp::mat_t<3, 3, wp::float32>> var_contact_frame_out,
    wp::array_t<wp::float32> var_contact_includemargin_out,
    wp::array_t<wp::vec_t<5, wp::float32>> var_contact_friction_out,
    wp::array_t<wp::vec_t<2, wp::float32>> var_contact_solref_out,
    wp::array_t<wp::vec_t<2, wp::float32>> var_contact_solreffriction_out,
    wp::array_t<wp::vec_t<5, wp::float32>> var_contact_solimp_out,
    wp::array_t<wp::int32> var_contact_dim_out,
    wp::array_t<wp::vec_t<2, wp::int32>> var_contact_geom_out,
    wp::array_t<wp::int32> var_contact_efc_address_out,
    wp::array_t<wp::int32> var_contact_worldid_out,
    wp::array_t<wp::int32> var_contact_type_out,
    wp::array_t<wp::int32> var_contact_geomcollisionid_out,
    wp::array_t<wp::int32> var_nacon_out)
{
    //---------
    // primal vars
    bool var_0;
    wp::float32 var_1;
    bool var_2;
    bool var_3;
    bool var_4;
    const wp::int32 var_5 = 0;
    wp::int32 var_6;
    const wp::int32 var_7 = -2;
    bool var_8;
    bool var_9;
    const wp::int32 var_10 = 1;
    wp::int32 var_11;
    const wp::int32 var_12 = -1;
    bool var_13;
    const wp::int32 var_14 = 0;
    const wp::int32 var_15 = 0;
    bool var_16;
    const wp::int32 var_17 = 0;
    wp::int32 var_18;
    const wp::int32 var_19 = -1;
    bool var_20;
    const wp::int32 var_21 = 1;
    wp::int32 var_22;
    wp::int32 var_23;
    const wp::int32 var_24 = 1;
    wp::int32 var_25;
    const wp::int32 var_26 = 0;
    bool var_27;
    const wp::int32 var_28 = 2;
    wp::int32 var_29;
    wp::int32 var_30;
    const wp::int32 var_31 = 0;
    const wp::int32 var_32 = 1;
    wp::int32 var_33;
    bool var_34;
    wp::float32 var_35;
    wp::shape_t* var_36;
    const wp::int32 var_37 = 1;
    wp::int32 var_38;
    wp::shape_t var_39;
    wp::range_t var_40;
    wp::int32 var_41;
    const wp::int32 var_42 = -1;
    wp::int32 var_43;
    const wp::int32 var_44 = 0;
    //---------
    // forward
    // def write_contact(                                                                     <L 214>
    // active = dist_in < margin_in                                                           <L 253>
    var_0 = (var_dist_in < var_margin_in);
    // detected = dist_in < margin_in + gap_in                                                <L 254>
    var_1 = wp::add(var_margin_in, var_gap_in);
    var_2 = (var_dist_in < var_1);
    // if (pairid_in[0] == -2 or not detected) and pairid_in[1] == -1:                        <L 257>
    var_6 = wp::extract(var_pairid_in, var_5);
    var_8 = (var_6 == var_7);
    var_4 = var_8;
    if (!var_4) {
        var_9 = wp::unot(var_2);
        var_4 = var_4 || var_9;
    }
    var_3 = var_4;
    if (var_3) {
        var_11 = wp::extract(var_pairid_in, var_10);
        var_13 = (var_11 == var_12);
        var_3 = var_3 && var_13;
    }
    if (var_3) {
        // return 0                                                                           <L 258>
        return var_14;
    }
    // contact_type = 0                                                                       <L 260>
    // if pairid_in[0] >= -1 and detected:                                                    <L 262>
    var_18 = wp::extract(var_pairid_in, var_17);
    var_20 = (var_18 >= var_19);
    var_16 = var_20;
    if (var_16) {
        var_16 = var_16 && var_2;
    }
    if (var_16) {
        // contact_type |= ContactType.CONSTRAINT                                             <L 263>
        var_22 = wp::bit_or(var_15, var_21);
    }
    var_23 = wp::where(var_16, var_22, var_15);
    // if pairid_in[1] >= 0:                                                                  <L 265>
    var_25 = wp::extract(var_pairid_in, var_24);
    var_27 = (var_25 >= var_26);
    if (var_27) {
        // contact_type |= ContactType.SENSOR                                                 <L 266>
        var_29 = wp::bit_or(var_23, var_28);
    }
    var_30 = wp::where(var_27, var_29, var_23);
    // cid = wp.atomic_add(nacon_out, 0, 1)                                                   <L 268>
    var_33 = wp::atomic_add(var_nacon_out, var_31, var_32);
    // if cid < naconmax_in:                                                                  <L 269>
    var_34 = (var_33 < var_naconmax_in);
    if (var_34) {
        // contact_dist_out[cid] = dist_in                                                    <L 270>
        wp::array_store(var_contact_dist_out, var_33, var_dist_in);
        // contact_pos_out[cid] = pos_in                                                      <L 271>
        wp::array_store(var_contact_pos_out, var_33, var_pos_in);
        // contact_frame_out[cid] = frame_in                                                  <L 272>
        wp::array_store(var_contact_frame_out, var_33, var_frame_in);
        // contact_geom_out[cid] = geoms_in                                                   <L 273>
        wp::array_store(var_contact_geom_out, var_33, var_geoms_in);
        // contact_worldid_out[cid] = worldid_in                                              <L 274>
        wp::array_store(var_contact_worldid_out, var_33, var_worldid_in);
        // includemargin = margin_in                                                          <L 275>
        var_35 = wp::copy(var_margin_in);
        // contact_includemargin_out[cid] = includemargin                                     <L 276>
        wp::array_store(var_contact_includemargin_out, var_33, var_35);
        // contact_dim_out[cid] = condim_in                                                   <L 277>
        wp::array_store(var_contact_dim_out, var_33, var_condim_in);
        // contact_friction_out[cid] = friction_in                                            <L 278>
        wp::array_store(var_contact_friction_out, var_33, var_friction_in);
        // contact_solref_out[cid] = solref_in                                                <L 279>
        wp::array_store(var_contact_solref_out, var_33, var_solref_in);
        // contact_solreffriction_out[cid] = solreffriction_in                                <L 280>
        wp::array_store(var_contact_solreffriction_out, var_33, var_solreffriction_in);
        // contact_solimp_out[cid] = solimp_in                                                <L 281>
        wp::array_store(var_contact_solimp_out, var_33, var_solimp_in);
        // contact_type_out[cid] = contact_type                                               <L 282>
        wp::array_store(var_contact_type_out, var_33, var_30);
        // contact_geomcollisionid_out[cid] = id_                                             <L 283>
        wp::array_store(var_contact_geomcollisionid_out, var_33, var_id_);
        // for i in range(contact_efc_address_out.shape[1]):                                  <L 284>
        var_36 = &(var_contact_efc_address_out.shape);
        var_39 = wp::load(var_36);
        var_38 = wp::extract(var_39, var_37);
        var_40 = wp::range(var_38);
        start_for_1:;
            if (iter_cmp(var_40) == 0) goto end_for_1;
            var_41 = wp::iter_next(var_40);
            // contact_efc_address_out[cid, i] = -1                                           <L 285>
            wp::array_store(var_contact_efc_address_out, var_33, var_41, var_42);
            goto start_for_1;
        end_for_1:;
        // return int(active)                                                                 <L 286>
        var_43 = wp::int(var_0);
        return var_43;
    }
    // return 0                                                                               <L 287>
    return var_44;
}


// /home/cofallen/miniconda3/envs/mjlab/lib/python3.12/site-packages/mujoco_warp/_src/collision_primitive.py:963
static CUDA_CALLABLE void plane_cylinder_wrapper_0(
    wp::int32 var_naconmax_in,
    Geom_3242f8a8 var_plane,
    Geom_3242f8a8 var_cylinder,
    wp::int32 var_worldid,
    wp::float32 var_margin,
    wp::float32 var_gap,
    wp::int32 var_condim,
    wp::vec_t<5, wp::float32> var_friction,
    wp::vec_t<2, wp::float32> var_solref,
    wp::vec_t<2, wp::float32> var_solreffriction,
    wp::vec_t<5, wp::float32> var_solimp,
    wp::vec_t<2, wp::int32> var_geoms,
    wp::vec_t<2, wp::int32> var_pairid,
    wp::array_t<wp::float32> var_contact_dist_out,
    wp::array_t<wp::vec_t<3, wp::float32>> var_contact_pos_out,
    wp::array_t<wp::mat_t<3, 3, wp::float32>> var_contact_frame_out,
    wp::array_t<wp::float32> var_contact_includemargin_out,
    wp::array_t<wp::vec_t<5, wp::float32>> var_contact_friction_out,
    wp::array_t<wp::vec_t<2, wp::float32>> var_contact_solref_out,
    wp::array_t<wp::vec_t<2, wp::float32>> var_contact_solreffriction_out,
    wp::array_t<wp::vec_t<5, wp::float32>> var_contact_solimp_out,
    wp::array_t<wp::int32> var_contact_dim_out,
    wp::array_t<wp::vec_t<2, wp::int32>> var_contact_geom_out,
    wp::array_t<wp::int32> var_contact_efc_address_out,
    wp::array_t<wp::int32> var_contact_worldid_out,
    wp::array_t<wp::int32> var_contact_type_out,
    wp::array_t<wp::int32> var_contact_geomcollisionid_out,
    wp::array_t<wp::int32> var_nacon_out)
{
    //---------
    // primal vars
    wp::mat_t<3, 3, wp::float32>* var_0;
    const wp::int32 var_1 = 0;
    const wp::int32 var_2 = 2;
    wp::float32 var_3;
    wp::mat_t<3, 3, wp::float32> var_4;
    wp::mat_t<3, 3, wp::float32>* var_5;
    const wp::int32 var_6 = 1;
    const wp::int32 var_7 = 2;
    wp::float32 var_8;
    wp::mat_t<3, 3, wp::float32> var_9;
    wp::mat_t<3, 3, wp::float32>* var_10;
    const wp::int32 var_11 = 2;
    const wp::int32 var_12 = 2;
    wp::float32 var_13;
    wp::mat_t<3, 3, wp::float32> var_14;
    wp::vec_t<3, wp::float32> var_15;
    wp::vec_t<3, wp::float32>* var_16;
    wp::vec_t<3, wp::float32>* var_17;
    wp::vec_t<3, wp::float32>* var_18;
    wp::vec_t<3, wp::float32>* var_19;
    const wp::int32 var_20 = 0;
    wp::float32 var_21;
    wp::vec_t<3, wp::float32> var_22;
    wp::vec_t<3, wp::float32>* var_23;
    const wp::int32 var_24 = 1;
    wp::float32 var_25;
    wp::vec_t<3, wp::float32> var_26;
    wp::vec_t<4, wp::float32> var_27;
    wp::mat_t<4, 3, wp::float32> var_28;
    wp::vec_t<3, wp::float32> var_29;
    wp::vec_t<3, wp::float32> var_30;
    wp::vec_t<3, wp::float32> var_31;
    wp::vec_t<3, wp::float32> var_32;
    wp::mat_t<3, 3, wp::float32> var_33;
    const wp::int32 var_34 = 0;
    wp::float32 var_35;
    wp::vec_t<3, wp::float32> var_36;
    wp::int32 var_37;
    const wp::int32 var_38 = 1;
    wp::float32 var_39;
    wp::vec_t<3, wp::float32> var_40;
    wp::int32 var_41;
    const wp::int32 var_42 = 2;
    wp::float32 var_43;
    wp::vec_t<3, wp::float32> var_44;
    wp::int32 var_45;
    const wp::int32 var_46 = 3;
    wp::float32 var_47;
    wp::vec_t<3, wp::float32> var_48;
    wp::int32 var_49;
    //---------
    // forward
    // def plane_cylinder_wrapper(                                                            <L 964>
    // cylinder_axis = wp.vec3(cylinder.rot[0, 2], cylinder.rot[1, 2], cylinder.rot[2, 2])       <L 999>
    var_0 = &((var_cylinder).rot);
    var_4 = wp::load(var_0);
    var_3 = wp::extract(var_4, var_1, var_2);
    var_5 = &((var_cylinder).rot);
    var_9 = wp::load(var_5);
    var_8 = wp::extract(var_9, var_6, var_7);
    var_10 = &((var_cylinder).rot);
    var_14 = wp::load(var_10);
    var_13 = wp::extract(var_14, var_11, var_12);
    var_15 = wp::vec_t<3, wp::float32>(var_3, var_8, var_13);
    // dist, pos, normal = plane_cylinder(                                                    <L 1001>
    // plane.normal,                                                                          <L 1002>
    var_16 = &((var_plane).normal);
    // plane.pos,                                                                             <L 1003>
    var_17 = &((var_plane).pos);
    // cylinder.pos,                                                                          <L 1004>
    var_18 = &((var_cylinder).pos);
    // cylinder_axis,                                                                         <L 1005>
    // cylinder.size[0],  # radius                                                            <L 1006>
    var_19 = &((var_cylinder).size);
    var_22 = wp::load(var_19);
    var_21 = wp::extract(var_22, var_20);
    // cylinder.size[1],  # half_height                                                       <L 1007>
    var_23 = &((var_cylinder).size);
    var_26 = wp::load(var_23);
    var_25 = wp::extract(var_26, var_24);
    var_30 = wp::load(var_16);
    var_31 = wp::load(var_17);
    var_32 = wp::load(var_18);
    plane_cylinder_0(var_30, var_31, var_32, var_15, var_21, var_25, var_27, var_28, var_29);
    // frame = make_frame(normal)                                                             <L 1010>
    var_33 = make_frame_0(var_29);
    // for i in range(4):                                                                     <L 1011>
    // write_contact(                                                                         <L 1012>
    // naconmax_in,                                                                           <L 1013>
    // i,                                                                                     <L 1014>
    // dist[i],                                                                               <L 1015>
    var_35 = wp::extract(var_27, var_34);
    // pos[i],                                                                                <L 1016>
    var_36 = wp::extract(var_28, var_34);
    // frame,                                                                                 <L 1017>
    // margin,                                                                                <L 1018>
    // gap,                                                                                   <L 1019>
    // condim,                                                                                <L 1020>
    // friction,                                                                              <L 1021>
    // solref,                                                                                <L 1022>
    // solreffriction,                                                                        <L 1023>
    // solimp,                                                                                <L 1024>
    // geoms,                                                                                 <L 1025>
    // pairid,                                                                                <L 1026>
    // worldid,                                                                               <L 1027>
    // contact_dist_out,                                                                      <L 1028>
    // contact_pos_out,                                                                       <L 1029>
    // contact_frame_out,                                                                     <L 1030>
    // contact_includemargin_out,                                                             <L 1031>
    // contact_friction_out,                                                                  <L 1032>
    // contact_solref_out,                                                                    <L 1033>
    // contact_solreffriction_out,                                                            <L 1034>
    // contact_solimp_out,                                                                    <L 1035>
    // contact_dim_out,                                                                       <L 1036>
    // contact_geom_out,                                                                      <L 1037>
    // contact_efc_address_out,                                                               <L 1038>
    // contact_worldid_out,                                                                   <L 1039>
    // contact_type_out,                                                                      <L 1040>
    // contact_geomcollisionid_out,                                                           <L 1041>
    // nacon_out,                                                                             <L 1042>
    var_37 = write_contact_0(var_naconmax_in, var_34, var_35, var_36, var_33, var_margin, var_gap, var_condim, var_friction, var_solref, var_solreffriction, var_solimp, var_geoms, var_pairid, var_worldid, var_contact_dist_out, var_contact_pos_out, var_contact_frame_out, var_contact_includemargin_out, var_contact_friction_out, var_contact_solref_out, var_contact_solreffriction_out, var_contact_solimp_out, var_contact_dim_out, var_contact_geom_out, var_contact_efc_address_out, var_contact_worldid_out, var_contact_type_out, var_contact_geomcollisionid_out, var_nacon_out);
    // write_contact(                                                                         <L 1012>
    // naconmax_in,                                                                           <L 1013>
    // i,                                                                                     <L 1014>
    // dist[i],                                                                               <L 1015>
    var_39 = wp::extract(var_27, var_38);
    // pos[i],                                                                                <L 1016>
    var_40 = wp::extract(var_28, var_38);
    // frame,                                                                                 <L 1017>
    // margin,                                                                                <L 1018>
    // gap,                                                                                   <L 1019>
    // condim,                                                                                <L 1020>
    // friction,                                                                              <L 1021>
    // solref,                                                                                <L 1022>
    // solreffriction,                                                                        <L 1023>
    // solimp,                                                                                <L 1024>
    // geoms,                                                                                 <L 1025>
    // pairid,                                                                                <L 1026>
    // worldid,                                                                               <L 1027>
    // contact_dist_out,                                                                      <L 1028>
    // contact_pos_out,                                                                       <L 1029>
    // contact_frame_out,                                                                     <L 1030>
    // contact_includemargin_out,                                                             <L 1031>
    // contact_friction_out,                                                                  <L 1032>
    // contact_solref_out,                                                                    <L 1033>
    // contact_solreffriction_out,                                                            <L 1034>
    // contact_solimp_out,                                                                    <L 1035>
    // contact_dim_out,                                                                       <L 1036>
    // contact_geom_out,                                                                      <L 1037>
    // contact_efc_address_out,                                                               <L 1038>
    // contact_worldid_out,                                                                   <L 1039>
    // contact_type_out,                                                                      <L 1040>
    // contact_geomcollisionid_out,                                                           <L 1041>
    // nacon_out,                                                                             <L 1042>
    var_41 = write_contact_0(var_naconmax_in, var_38, var_39, var_40, var_33, var_margin, var_gap, var_condim, var_friction, var_solref, var_solreffriction, var_solimp, var_geoms, var_pairid, var_worldid, var_contact_dist_out, var_contact_pos_out, var_contact_frame_out, var_contact_includemargin_out, var_contact_friction_out, var_contact_solref_out, var_contact_solreffriction_out, var_contact_solimp_out, var_contact_dim_out, var_contact_geom_out, var_contact_efc_address_out, var_contact_worldid_out, var_contact_type_out, var_contact_geomcollisionid_out, var_nacon_out);
    // write_contact(                                                                         <L 1012>
    // naconmax_in,                                                                           <L 1013>
    // i,                                                                                     <L 1014>
    // dist[i],                                                                               <L 1015>
    var_43 = wp::extract(var_27, var_42);
    // pos[i],                                                                                <L 1016>
    var_44 = wp::extract(var_28, var_42);
    // frame,                                                                                 <L 1017>
    // margin,                                                                                <L 1018>
    // gap,                                                                                   <L 1019>
    // condim,                                                                                <L 1020>
    // friction,                                                                              <L 1021>
    // solref,                                                                                <L 1022>
    // solreffriction,                                                                        <L 1023>
    // solimp,                                                                                <L 1024>
    // geoms,                                                                                 <L 1025>
    // pairid,                                                                                <L 1026>
    // worldid,                                                                               <L 1027>
    // contact_dist_out,                                                                      <L 1028>
    // contact_pos_out,                                                                       <L 1029>
    // contact_frame_out,                                                                     <L 1030>
    // contact_includemargin_out,                                                             <L 1031>
    // contact_friction_out,                                                                  <L 1032>
    // contact_solref_out,                                                                    <L 1033>
    // contact_solreffriction_out,                                                            <L 1034>
    // contact_solimp_out,                                                                    <L 1035>
    // contact_dim_out,                                                                       <L 1036>
    // contact_geom_out,                                                                      <L 1037>
    // contact_efc_address_out,                                                               <L 1038>
    // contact_worldid_out,                                                                   <L 1039>
    // contact_type_out,                                                                      <L 1040>
    // contact_geomcollisionid_out,                                                           <L 1041>
    // nacon_out,                                                                             <L 1042>
    var_45 = write_contact_0(var_naconmax_in, var_42, var_43, var_44, var_33, var_margin, var_gap, var_condim, var_friction, var_solref, var_solreffriction, var_solimp, var_geoms, var_pairid, var_worldid, var_contact_dist_out, var_contact_pos_out, var_contact_frame_out, var_contact_includemargin_out, var_contact_friction_out, var_contact_solref_out, var_contact_solreffriction_out, var_contact_solimp_out, var_contact_dim_out, var_contact_geom_out, var_contact_efc_address_out, var_contact_worldid_out, var_contact_type_out, var_contact_geomcollisionid_out, var_nacon_out);
    // write_contact(                                                                         <L 1012>
    // naconmax_in,                                                                           <L 1013>
    // i,                                                                                     <L 1014>
    // dist[i],                                                                               <L 1015>
    var_47 = wp::extract(var_27, var_46);
    // pos[i],                                                                                <L 1016>
    var_48 = wp::extract(var_28, var_46);
    // frame,                                                                                 <L 1017>
    // margin,                                                                                <L 1018>
    // gap,                                                                                   <L 1019>
    // condim,                                                                                <L 1020>
    // friction,                                                                              <L 1021>
    // solref,                                                                                <L 1022>
    // solreffriction,                                                                        <L 1023>
    // solimp,                                                                                <L 1024>
    // geoms,                                                                                 <L 1025>
    // pairid,                                                                                <L 1026>
    // worldid,                                                                               <L 1027>
    // contact_dist_out,                                                                      <L 1028>
    // contact_pos_out,                                                                       <L 1029>
    // contact_frame_out,                                                                     <L 1030>
    // contact_includemargin_out,                                                             <L 1031>
    // contact_friction_out,                                                                  <L 1032>
    // contact_solref_out,                                                                    <L 1033>
    // contact_solreffriction_out,                                                            <L 1034>
    // contact_solimp_out,                                                                    <L 1035>
    // contact_dim_out,                                                                       <L 1036>
    // contact_geom_out,                                                                      <L 1037>
    // contact_efc_address_out,                                                               <L 1038>
    // contact_worldid_out,                                                                   <L 1039>
    // contact_type_out,                                                                      <L 1040>
    // contact_geomcollisionid_out,                                                           <L 1041>
    // nacon_out,                                                                             <L 1042>
    var_49 = write_contact_0(var_naconmax_in, var_46, var_47, var_48, var_33, var_margin, var_gap, var_condim, var_friction, var_solref, var_solreffriction, var_solimp, var_geoms, var_pairid, var_worldid, var_contact_dist_out, var_contact_pos_out, var_contact_frame_out, var_contact_includemargin_out, var_contact_friction_out, var_contact_solref_out, var_contact_solreffriction_out, var_contact_solimp_out, var_contact_dim_out, var_contact_geom_out, var_contact_efc_address_out, var_contact_worldid_out, var_contact_type_out, var_contact_geomcollisionid_out, var_nacon_out);
}


// /home/cofallen/miniconda3/envs/mjlab/lib/python3.12/site-packages/mujoco_warp/_src/collision_primitive.py:51
static CUDA_CALLABLE void plane_convex_0(
    wp::vec_t<3, wp::float32> var_plane_normal,
    wp::vec_t<3, wp::float32> var_plane_pos,
    Geom_3242f8a8 var_convex,
    wp::vec_t<4, wp::float32> & ret_0,
    wp::mat_t<4, 3, wp::float32> & ret_1,
    wp::vec_t<3, wp::float32> & ret_2)
{
    //---------
    // primal vars
    const wp::float32 var_0 = 1000000.0;
    const wp::float32 var_1 = 10000000000.0;
    wp::vec_t<4, wp::float32> var_2;
    wp::mat_t<4, 3, wp::float32> var_3;
    const wp::int32 var_4 = 0;
    wp::int32 var_5;
    wp::mat_t<3, 3, wp::float32>* var_6;
    wp::mat_t<3, 3, wp::float32> var_7;
    wp::mat_t<3, 3, wp::float32> var_8;
    wp::vec_t<3, wp::float32>* var_9;
    wp::vec_t<3, wp::float32> var_10;
    wp::vec_t<3, wp::float32> var_11;
    wp::vec_t<3, wp::float32> var_12;
    wp::mat_t<3, 3, wp::float32>* var_13;
    wp::mat_t<3, 3, wp::float32> var_14;
    wp::mat_t<3, 3, wp::float32> var_15;
    wp::vec_t<3, wp::float32> var_16;
    const wp::int32 var_17 = -1;
    const wp::int32 var_18 = -1;
    const wp::int32 var_19 = -1;
    const wp::int32 var_20 = -1;
    wp::vec_t<4, wp::int32> var_21;
    bool var_22;
    wp::int32* var_23;
    const wp::int32 var_24 = -1;
    bool var_25;
    wp::int32 var_26;
    wp::int32* var_27;
    const wp::int32 var_28 = 10;
    bool var_29;
    wp::int32 var_30;
    const wp::float32 var_31 = -1000000.0;
    wp::float32 var_32;
    wp::vec_t<3, wp::float32> var_33;
    wp::int32* var_34;
    wp::range_t var_35;
    wp::int32 var_36;
    wp::int32 var_37;
    wp::array_t<wp::vec_t<3, wp::float32>>* var_38;
    wp::int32* var_39;
    wp::int32 var_40;
    wp::int32 var_41;
    wp::vec_t<3, wp::float32>* var_42;
    wp::array_t<wp::vec_t<3, wp::float32>> var_43;
    wp::vec_t<3, wp::float32> var_44;
    wp::vec_t<3, wp::float32> var_45;
    wp::vec_t<3, wp::float32> var_46;
    wp::float32 var_47;
    bool var_48;
    wp::float32 var_49;
    const wp::int32 var_50 = 0;
    wp::vec_t<3, wp::float32> var_51;
    wp::float32 var_52;
    wp::vec_t<3, wp::float32> var_53;
    const wp::int32 var_54 = 0;
    bool var_55;
    const wp::float32 var_56 = 0.001;
    wp::float32 var_57;
    const wp::float32 var_58 = -1000000.0;
    wp::float32 var_59;
    wp::vec_t<3, wp::float32> var_60;
    wp::int32* var_61;
    wp::range_t var_62;
    wp::int32 var_63;
    wp::int32 var_64;
    wp::array_t<wp::vec_t<3, wp::float32>>* var_65;
    wp::int32* var_66;
    wp::int32 var_67;
    wp::int32 var_68;
    wp::vec_t<3, wp::float32>* var_69;
    wp::array_t<wp::vec_t<3, wp::float32>> var_70;
    wp::vec_t<3, wp::float32> var_71;
    wp::vec_t<3, wp::float32> var_72;
    wp::vec_t<3, wp::float32> var_73;
    wp::float32 var_74;
    bool var_75;
    const wp::float32 var_76 = 0.0;
    const wp::float32 var_77 = -1000000.0;
    wp::float32 var_78;
    wp::vec_t<3, wp::float32> var_79;
    wp::float32 var_80;
    wp::float32 var_81;
    bool var_82;
    const wp::int32 var_83 = 1;
    wp::float32 var_84;
    wp::vec_t<3, wp::float32> var_85;
    wp::float32 var_86;
    wp::vec_t<3, wp::float32> var_87;
    wp::vec_t<3, wp::float32> var_88;
    wp::vec_t<3, wp::float32> var_89;
    const wp::float32 var_90 = -1000000.0;
    wp::float32 var_91;
    wp::vec_t<3, wp::float32> var_92;
    wp::int32* var_93;
    wp::range_t var_94;
    wp::int32 var_95;
    wp::int32 var_96;
    wp::array_t<wp::vec_t<3, wp::float32>>* var_97;
    wp::int32* var_98;
    wp::int32 var_99;
    wp::int32 var_100;
    wp::vec_t<3, wp::float32>* var_101;
    wp::array_t<wp::vec_t<3, wp::float32>> var_102;
    wp::vec_t<3, wp::float32> var_103;
    wp::vec_t<3, wp::float32> var_104;
    wp::vec_t<3, wp::float32> var_105;
    wp::float32 var_106;
    bool var_107;
    const wp::float32 var_108 = 0.0;
    const wp::float32 var_109 = -1000000.0;
    wp::float32 var_110;
    wp::vec_t<3, wp::float32> var_111;
    wp::float32 var_112;
    wp::float32 var_113;
    wp::float32 var_114;
    bool var_115;
    const wp::int32 var_116 = 2;
    wp::float32 var_117;
    wp::vec_t<3, wp::float32> var_118;
    wp::float32 var_119;
    wp::vec_t<3, wp::float32> var_120;
    wp::vec_t<3, wp::float32> var_121;
    wp::vec_t<3, wp::float32> var_122;
    wp::vec_t<3, wp::float32> var_123;
    wp::vec_t<3, wp::float32> var_124;
    const wp::float32 var_125 = -1000000.0;
    wp::float32 var_126;
    wp::int32* var_127;
    wp::range_t var_128;
    wp::int32 var_129;
    wp::int32 var_130;
    wp::array_t<wp::vec_t<3, wp::float32>>* var_131;
    wp::int32* var_132;
    wp::int32 var_133;
    wp::int32 var_134;
    wp::vec_t<3, wp::float32>* var_135;
    wp::array_t<wp::vec_t<3, wp::float32>> var_136;
    wp::vec_t<3, wp::float32> var_137;
    wp::vec_t<3, wp::float32> var_138;
    wp::vec_t<3, wp::float32> var_139;
    wp::float32 var_140;
    bool var_141;
    const wp::float32 var_142 = 0.0;
    const wp::float32 var_143 = -1000000.0;
    wp::float32 var_144;
    wp::vec_t<3, wp::float32> var_145;
    wp::vec_t<3, wp::float32> var_146;
    wp::float32 var_147;
    wp::float32 var_148;
    wp::float32 var_149;
    wp::float32 var_150;
    wp::float32 var_151;
    wp::float32 var_152;
    wp::float32 var_153;
    bool var_154;
    const wp::int32 var_155 = 3;
    wp::float32 var_156;
    wp::float32 var_157;
    wp::array_t<wp::int32>* var_158;
    wp::int32* var_159;
    wp::int32 var_160;
    wp::int32* var_161;
    wp::array_t<wp::int32> var_162;
    wp::int32 var_163;
    wp::int32 var_164;
    wp::int32* var_165;
    const wp::int32 var_166 = 2;
    wp::int32 var_167;
    wp::int32 var_168;
    wp::int32* var_169;
    const wp::int32 var_170 = 2;
    wp::int32 var_171;
    wp::int32 var_172;
    wp::int32 var_173;
    wp::int32* var_174;
    const wp::int32 var_175 = 2;
    wp::int32 var_176;
    wp::int32 var_177;
    const wp::int32 var_178 = 2;
    wp::int32 var_179;
    wp::int32 var_180;
    const wp::float32 var_181 = -1000000.0;
    wp::float32 var_182;
    const wp::int32 var_183 = -1;
    wp::int32 var_184;
    const wp::int32 var_185 = 0;
    wp::int32 var_186;
    const bool var_187 = true;
    wp::int32 var_188;
    wp::array_t<wp::int32>* var_189;
    wp::int32 var_190;
    wp::int32* var_191;
    wp::array_t<wp::int32> var_192;
    wp::int32 var_193;
    wp::int32 var_194;
    wp::array_t<wp::int32>* var_195;
    wp::int32 var_196;
    wp::int32* var_197;
    wp::array_t<wp::int32> var_198;
    const wp::int32 var_199 = 0;
    bool var_200;
    wp::int32 var_201;
    wp::array_t<wp::int32>* var_202;
    wp::int32 var_203;
    wp::int32* var_204;
    wp::array_t<wp::int32> var_205;
    wp::int32 var_206;
    wp::int32 var_207;
    wp::array_t<wp::int32>* var_208;
    wp::int32 var_209;
    wp::int32* var_210;
    wp::array_t<wp::int32> var_211;
    wp::int32 var_212;
    wp::int32 var_213;
    wp::array_t<wp::vec_t<3, wp::float32>>* var_214;
    wp::int32* var_215;
    wp::int32 var_216;
    wp::int32 var_217;
    wp::vec_t<3, wp::float32>* var_218;
    wp::array_t<wp::vec_t<3, wp::float32>> var_219;
    wp::vec_t<3, wp::float32> var_220;
    wp::vec_t<3, wp::float32> var_221;
    wp::float32 var_222;
    bool var_223;
    wp::float32 var_224;
    wp::int32 var_225;
    wp::float32 var_226;
    wp::int32 var_227;
    const wp::int32 var_228 = 1;
    wp::int32 var_229;
    wp::int32 var_230;
    bool var_231;
    wp::int32 var_232;
    const wp::float32 var_233 = 0.0;
    const wp::float32 var_234 = 0.001;
    wp::float32 var_235;
    wp::float32 var_236;
    const wp::float32 var_237 = -1000000.0;
    wp::float32 var_238;
    const bool var_239 = true;
    wp::int32 var_240;
    wp::array_t<wp::int32>* var_241;
    wp::int32 var_242;
    wp::int32* var_243;
    wp::array_t<wp::int32> var_244;
    wp::int32 var_245;
    wp::int32 var_246;
    wp::array_t<wp::int32>* var_247;
    wp::int32 var_248;
    wp::int32* var_249;
    wp::array_t<wp::int32> var_250;
    const wp::int32 var_251 = 0;
    bool var_252;
    wp::int32 var_253;
    wp::array_t<wp::int32>* var_254;
    wp::int32 var_255;
    wp::int32* var_256;
    wp::array_t<wp::int32> var_257;
    wp::int32 var_258;
    wp::int32 var_259;
    wp::array_t<wp::int32>* var_260;
    wp::int32 var_261;
    wp::int32* var_262;
    wp::array_t<wp::int32> var_263;
    wp::int32 var_264;
    wp::int32 var_265;
    wp::array_t<wp::vec_t<3, wp::float32>>* var_266;
    wp::int32* var_267;
    wp::int32 var_268;
    wp::int32 var_269;
    wp::vec_t<3, wp::float32>* var_270;
    wp::array_t<wp::vec_t<3, wp::float32>> var_271;
    wp::vec_t<3, wp::float32> var_272;
    wp::vec_t<3, wp::float32> var_273;
    wp::float32 var_274;
    bool var_275;
    const wp::float32 var_276 = -1000000.0;
    wp::float32 var_277;
    bool var_278;
    wp::float32 var_279;
    wp::int32 var_280;
    wp::int32 var_281;
    wp::float32 var_282;
    const wp::int32 var_283 = 1;
    wp::int32 var_284;
    wp::int32 var_285;
    bool var_286;
    wp::int32 var_287;
    wp::int32 var_288;
    wp::array_t<wp::int32>* var_289;
    wp::int32 var_290;
    wp::int32* var_291;
    wp::array_t<wp::int32> var_292;
    wp::int32 var_293;
    wp::int32 var_294;
    wp::array_t<wp::vec_t<3, wp::float32>>* var_295;
    wp::int32* var_296;
    wp::int32 var_297;
    wp::int32 var_298;
    wp::vec_t<3, wp::float32>* var_299;
    wp::array_t<wp::vec_t<3, wp::float32>> var_300;
    wp::vec_t<3, wp::float32> var_301;
    wp::vec_t<3, wp::float32> var_302;
    const wp::int32 var_303 = 0;
    const wp::float32 var_304 = -1000000.0;
    wp::float32 var_305;
    const bool var_306 = true;
    wp::int32 var_307;
    wp::array_t<wp::int32>* var_308;
    wp::int32 var_309;
    wp::int32* var_310;
    wp::array_t<wp::int32> var_311;
    wp::int32 var_312;
    wp::int32 var_313;
    wp::array_t<wp::int32>* var_314;
    wp::int32 var_315;
    wp::int32* var_316;
    wp::array_t<wp::int32> var_317;
    const wp::int32 var_318 = 0;
    bool var_319;
    wp::int32 var_320;
    wp::array_t<wp::int32>* var_321;
    wp::int32 var_322;
    wp::int32* var_323;
    wp::array_t<wp::int32> var_324;
    wp::int32 var_325;
    wp::int32 var_326;
    wp::array_t<wp::int32>* var_327;
    wp::int32 var_328;
    wp::int32* var_329;
    wp::array_t<wp::int32> var_330;
    wp::int32 var_331;
    wp::int32 var_332;
    wp::array_t<wp::vec_t<3, wp::float32>>* var_333;
    wp::int32* var_334;
    wp::int32 var_335;
    wp::int32 var_336;
    wp::vec_t<3, wp::float32>* var_337;
    wp::array_t<wp::vec_t<3, wp::float32>> var_338;
    wp::vec_t<3, wp::float32> var_339;
    wp::vec_t<3, wp::float32> var_340;
    wp::float32 var_341;
    bool var_342;
    const wp::float32 var_343 = 0.0;
    const wp::float32 var_344 = -1000000.0;
    wp::float32 var_345;
    wp::array_t<wp::vec_t<3, wp::float32>>* var_346;
    wp::int32* var_347;
    wp::int32 var_348;
    wp::int32 var_349;
    wp::vec_t<3, wp::float32>* var_350;
    wp::array_t<wp::vec_t<3, wp::float32>> var_351;
    wp::vec_t<3, wp::float32> var_352;
    wp::vec_t<3, wp::float32> var_353;
    wp::float32 var_354;
    wp::float32 var_355;
    bool var_356;
    wp::float32 var_357;
    wp::int32 var_358;
    wp::int32 var_359;
    wp::float32 var_360;
    const wp::int32 var_361 = 1;
    wp::int32 var_362;
    wp::int32 var_363;
    bool var_364;
    wp::int32 var_365;
    wp::int32 var_366;
    wp::array_t<wp::int32>* var_367;
    wp::int32 var_368;
    wp::int32* var_369;
    wp::array_t<wp::int32> var_370;
    wp::int32 var_371;
    wp::int32 var_372;
    wp::array_t<wp::vec_t<3, wp::float32>>* var_373;
    wp::int32* var_374;
    wp::int32 var_375;
    wp::int32 var_376;
    wp::vec_t<3, wp::float32>* var_377;
    wp::array_t<wp::vec_t<3, wp::float32>> var_378;
    wp::vec_t<3, wp::float32> var_379;
    wp::vec_t<3, wp::float32> var_380;
    const wp::int32 var_381 = 1;
    wp::vec_t<3, wp::float32> var_382;
    wp::vec_t<3, wp::float32> var_383;
    const wp::float32 var_384 = -1000000.0;
    wp::float32 var_385;
    const bool var_386 = true;
    wp::int32 var_387;
    wp::array_t<wp::int32>* var_388;
    wp::int32 var_389;
    wp::int32* var_390;
    wp::array_t<wp::int32> var_391;
    wp::int32 var_392;
    wp::int32 var_393;
    wp::array_t<wp::int32>* var_394;
    wp::int32 var_395;
    wp::int32* var_396;
    wp::array_t<wp::int32> var_397;
    const wp::int32 var_398 = 0;
    bool var_399;
    wp::int32 var_400;
    wp::array_t<wp::int32>* var_401;
    wp::int32 var_402;
    wp::int32* var_403;
    wp::array_t<wp::int32> var_404;
    wp::int32 var_405;
    wp::int32 var_406;
    wp::array_t<wp::int32>* var_407;
    wp::int32 var_408;
    wp::int32* var_409;
    wp::array_t<wp::int32> var_410;
    wp::int32 var_411;
    wp::int32 var_412;
    wp::array_t<wp::vec_t<3, wp::float32>>* var_413;
    wp::int32* var_414;
    wp::int32 var_415;
    wp::int32 var_416;
    wp::vec_t<3, wp::float32>* var_417;
    wp::array_t<wp::vec_t<3, wp::float32>> var_418;
    wp::vec_t<3, wp::float32> var_419;
    wp::vec_t<3, wp::float32> var_420;
    wp::float32 var_421;
    bool var_422;
    const wp::float32 var_423 = 0.0;
    const wp::float32 var_424 = -1000000.0;
    wp::float32 var_425;
    wp::array_t<wp::vec_t<3, wp::float32>>* var_426;
    wp::int32* var_427;
    wp::int32 var_428;
    wp::int32 var_429;
    wp::vec_t<3, wp::float32>* var_430;
    wp::array_t<wp::vec_t<3, wp::float32>> var_431;
    wp::vec_t<3, wp::float32> var_432;
    wp::vec_t<3, wp::float32> var_433;
    wp::float32 var_434;
    wp::float32 var_435;
    wp::float32 var_436;
    bool var_437;
    wp::float32 var_438;
    wp::int32 var_439;
    wp::int32 var_440;
    wp::float32 var_441;
    const wp::int32 var_442 = 1;
    wp::int32 var_443;
    wp::int32 var_444;
    bool var_445;
    wp::int32 var_446;
    wp::int32 var_447;
    wp::array_t<wp::int32>* var_448;
    wp::int32 var_449;
    wp::int32* var_450;
    wp::array_t<wp::int32> var_451;
    wp::int32 var_452;
    wp::int32 var_453;
    wp::array_t<wp::vec_t<3, wp::float32>>* var_454;
    wp::int32* var_455;
    wp::int32 var_456;
    wp::int32 var_457;
    wp::vec_t<3, wp::float32>* var_458;
    wp::array_t<wp::vec_t<3, wp::float32>> var_459;
    wp::vec_t<3, wp::float32> var_460;
    wp::vec_t<3, wp::float32> var_461;
    const wp::int32 var_462 = 2;
    wp::vec_t<3, wp::float32> var_463;
    wp::vec_t<3, wp::float32> var_464;
    wp::vec_t<3, wp::float32> var_465;
    wp::vec_t<3, wp::float32> var_466;
    const wp::float32 var_467 = -1000000.0;
    wp::float32 var_468;
    const bool var_469 = true;
    wp::int32 var_470;
    wp::array_t<wp::int32>* var_471;
    wp::int32 var_472;
    wp::int32* var_473;
    wp::array_t<wp::int32> var_474;
    wp::int32 var_475;
    wp::int32 var_476;
    wp::array_t<wp::int32>* var_477;
    wp::int32 var_478;
    wp::int32* var_479;
    wp::array_t<wp::int32> var_480;
    const wp::int32 var_481 = 0;
    bool var_482;
    wp::int32 var_483;
    wp::array_t<wp::int32>* var_484;
    wp::int32 var_485;
    wp::int32* var_486;
    wp::array_t<wp::int32> var_487;
    wp::int32 var_488;
    wp::int32 var_489;
    wp::array_t<wp::int32>* var_490;
    wp::int32 var_491;
    wp::int32* var_492;
    wp::array_t<wp::int32> var_493;
    wp::int32 var_494;
    wp::int32 var_495;
    wp::array_t<wp::vec_t<3, wp::float32>>* var_496;
    wp::int32* var_497;
    wp::int32 var_498;
    wp::int32 var_499;
    wp::vec_t<3, wp::float32>* var_500;
    wp::array_t<wp::vec_t<3, wp::float32>> var_501;
    wp::vec_t<3, wp::float32> var_502;
    wp::vec_t<3, wp::float32> var_503;
    wp::float32 var_504;
    bool var_505;
    const wp::float32 var_506 = 0.0;
    const wp::float32 var_507 = -1000000.0;
    wp::float32 var_508;
    wp::array_t<wp::vec_t<3, wp::float32>>* var_509;
    wp::int32* var_510;
    wp::int32 var_511;
    wp::int32 var_512;
    wp::vec_t<3, wp::float32>* var_513;
    wp::array_t<wp::vec_t<3, wp::float32>> var_514;
    wp::vec_t<3, wp::float32> var_515;
    wp::vec_t<3, wp::float32> var_516;
    wp::array_t<wp::vec_t<3, wp::float32>>* var_517;
    wp::int32* var_518;
    wp::int32 var_519;
    wp::int32 var_520;
    wp::vec_t<3, wp::float32>* var_521;
    wp::array_t<wp::vec_t<3, wp::float32>> var_522;
    wp::vec_t<3, wp::float32> var_523;
    wp::vec_t<3, wp::float32> var_524;
    wp::float32 var_525;
    wp::float32 var_526;
    wp::float32 var_527;
    wp::float32 var_528;
    wp::float32 var_529;
    wp::float32 var_530;
    wp::float32 var_531;
    bool var_532;
    wp::float32 var_533;
    wp::int32 var_534;
    wp::int32 var_535;
    wp::float32 var_536;
    const wp::int32 var_537 = 1;
    wp::int32 var_538;
    wp::int32 var_539;
    bool var_540;
    wp::int32 var_541;
    wp::int32 var_542;
    wp::array_t<wp::int32>* var_543;
    wp::int32 var_544;
    wp::int32* var_545;
    wp::array_t<wp::int32> var_546;
    wp::int32 var_547;
    wp::int32 var_548;
    const wp::int32 var_549 = 3;
    wp::float32 var_550;
    wp::vec_t<3, wp::float32> var_551;
    wp::int32 var_552;
    wp::float32 var_553;
    wp::float32 var_554;
    wp::float32 var_555;
    wp::vec_t<3, wp::float32> var_556;
    wp::float32 var_557;
    wp::float32 var_558;
    wp::vec_t<3, wp::float32> var_559;
    wp::float32 var_560;
    wp::vec_t<3, wp::float32> var_561;
    wp::vec_t<3, wp::float32> var_562;
    wp::vec_t<3, wp::float32> var_563;
    wp::vec_t<3, wp::float32> var_564;
    wp::float32 var_565;
    wp::vec_t<3, wp::float32> var_566;
    wp::float32 var_567;
    wp::float32 var_568;
    const wp::int32 var_569 = 3;
    wp::int32 var_570;
    const wp::int32 var_571 = 0;
    wp::int32 var_572;
    const wp::int32 var_573 = 1;
    wp::int32 var_574;
    wp::range_t var_575;
    wp::int32 var_576;
    wp::int32 var_577;
    bool var_578;
    const wp::int32 var_579 = 1;
    wp::int32 var_580;
    wp::int32 var_581;
    const wp::int32 var_582 = 1;
    bool var_583;
    wp::array_t<wp::vec_t<3, wp::float32>>* var_584;
    wp::int32* var_585;
    wp::int32 var_586;
    wp::int32 var_587;
    wp::vec_t<3, wp::float32>* var_588;
    wp::array_t<wp::vec_t<3, wp::float32>> var_589;
    wp::vec_t<3, wp::float32> var_590;
    wp::vec_t<3, wp::float32> var_591;
    wp::vec_t<3, wp::float32>* var_592;
    wp::mat_t<3, 3, wp::float32>* var_593;
    wp::vec_t<3, wp::float32> var_594;
    wp::mat_t<3, 3, wp::float32> var_595;
    wp::vec_t<3, wp::float32> var_596;
    wp::vec_t<3, wp::float32> var_597;
    wp::array_t<wp::vec_t<3, wp::float32>>* var_598;
    wp::int32* var_599;
    wp::int32 var_600;
    wp::int32 var_601;
    wp::vec_t<3, wp::float32>* var_602;
    wp::array_t<wp::vec_t<3, wp::float32>> var_603;
    wp::vec_t<3, wp::float32> var_604;
    wp::vec_t<3, wp::float32> var_605;
    wp::float32 var_606;
    wp::float32 var_607;
    const wp::float32 var_608 = 0.5;
    wp::float32 var_609;
    wp::vec_t<3, wp::float32> var_610;
    wp::vec_t<3, wp::float32> var_611;
    const wp::int32 var_612 = 1;
    wp::int32 var_613;
    wp::int32 var_614;
    wp::float32 var_615;
    wp::float32 var_616;
    const wp::int32 var_617 = 2;
    wp::int32 var_618;
    const wp::int32 var_619 = 0;
    wp::int32 var_620;
    const wp::int32 var_621 = 1;
    wp::int32 var_622;
    wp::range_t var_623;
    wp::int32 var_624;
    wp::int32 var_625;
    bool var_626;
    const wp::int32 var_627 = 1;
    wp::int32 var_628;
    wp::int32 var_629;
    const wp::int32 var_630 = 1;
    bool var_631;
    wp::array_t<wp::vec_t<3, wp::float32>>* var_632;
    wp::int32* var_633;
    wp::int32 var_634;
    wp::int32 var_635;
    wp::vec_t<3, wp::float32>* var_636;
    wp::array_t<wp::vec_t<3, wp::float32>> var_637;
    wp::vec_t<3, wp::float32> var_638;
    wp::vec_t<3, wp::float32> var_639;
    wp::vec_t<3, wp::float32>* var_640;
    wp::mat_t<3, 3, wp::float32>* var_641;
    wp::vec_t<3, wp::float32> var_642;
    wp::mat_t<3, 3, wp::float32> var_643;
    wp::vec_t<3, wp::float32> var_644;
    wp::vec_t<3, wp::float32> var_645;
    wp::array_t<wp::vec_t<3, wp::float32>>* var_646;
    wp::int32* var_647;
    wp::int32 var_648;
    wp::int32 var_649;
    wp::vec_t<3, wp::float32>* var_650;
    wp::array_t<wp::vec_t<3, wp::float32>> var_651;
    wp::vec_t<3, wp::float32> var_652;
    wp::vec_t<3, wp::float32> var_653;
    wp::float32 var_654;
    wp::float32 var_655;
    const wp::float32 var_656 = 0.5;
    wp::float32 var_657;
    wp::vec_t<3, wp::float32> var_658;
    wp::vec_t<3, wp::float32> var_659;
    const wp::int32 var_660 = 1;
    wp::int32 var_661;
    wp::int32 var_662;
    wp::float32 var_663;
    wp::float32 var_664;
    wp::vec_t<3, wp::float32> var_665;
    const wp::int32 var_666 = 1;
    wp::int32 var_667;
    const wp::int32 var_668 = 0;
    wp::int32 var_669;
    const wp::int32 var_670 = 1;
    wp::int32 var_671;
    wp::range_t var_672;
    wp::int32 var_673;
    wp::int32 var_674;
    bool var_675;
    const wp::int32 var_676 = 1;
    wp::int32 var_677;
    wp::int32 var_678;
    const wp::int32 var_679 = 1;
    bool var_680;
    wp::array_t<wp::vec_t<3, wp::float32>>* var_681;
    wp::int32* var_682;
    wp::int32 var_683;
    wp::int32 var_684;
    wp::vec_t<3, wp::float32>* var_685;
    wp::array_t<wp::vec_t<3, wp::float32>> var_686;
    wp::vec_t<3, wp::float32> var_687;
    wp::vec_t<3, wp::float32> var_688;
    wp::vec_t<3, wp::float32>* var_689;
    wp::mat_t<3, 3, wp::float32>* var_690;
    wp::vec_t<3, wp::float32> var_691;
    wp::mat_t<3, 3, wp::float32> var_692;
    wp::vec_t<3, wp::float32> var_693;
    wp::vec_t<3, wp::float32> var_694;
    wp::array_t<wp::vec_t<3, wp::float32>>* var_695;
    wp::int32* var_696;
    wp::int32 var_697;
    wp::int32 var_698;
    wp::vec_t<3, wp::float32>* var_699;
    wp::array_t<wp::vec_t<3, wp::float32>> var_700;
    wp::vec_t<3, wp::float32> var_701;
    wp::vec_t<3, wp::float32> var_702;
    wp::float32 var_703;
    wp::float32 var_704;
    const wp::float32 var_705 = 0.5;
    wp::float32 var_706;
    wp::vec_t<3, wp::float32> var_707;
    wp::vec_t<3, wp::float32> var_708;
    const wp::int32 var_709 = 1;
    wp::int32 var_710;
    wp::int32 var_711;
    wp::float32 var_712;
    wp::float32 var_713;
    wp::vec_t<3, wp::float32> var_714;
    const wp::int32 var_715 = 0;
    wp::int32 var_716;
    const wp::int32 var_717 = 0;
    wp::int32 var_718;
    const wp::int32 var_719 = 1;
    wp::int32 var_720;
    wp::range_t var_721;
    wp::int32 var_722;
    wp::int32 var_723;
    bool var_724;
    const wp::int32 var_725 = 1;
    wp::int32 var_726;
    wp::int32 var_727;
    const wp::int32 var_728 = 1;
    bool var_729;
    wp::array_t<wp::vec_t<3, wp::float32>>* var_730;
    wp::int32* var_731;
    wp::int32 var_732;
    wp::int32 var_733;
    wp::vec_t<3, wp::float32>* var_734;
    wp::array_t<wp::vec_t<3, wp::float32>> var_735;
    wp::vec_t<3, wp::float32> var_736;
    wp::vec_t<3, wp::float32> var_737;
    wp::vec_t<3, wp::float32>* var_738;
    wp::mat_t<3, 3, wp::float32>* var_739;
    wp::vec_t<3, wp::float32> var_740;
    wp::mat_t<3, 3, wp::float32> var_741;
    wp::vec_t<3, wp::float32> var_742;
    wp::vec_t<3, wp::float32> var_743;
    wp::array_t<wp::vec_t<3, wp::float32>>* var_744;
    wp::int32* var_745;
    wp::int32 var_746;
    wp::int32 var_747;
    wp::vec_t<3, wp::float32>* var_748;
    wp::array_t<wp::vec_t<3, wp::float32>> var_749;
    wp::vec_t<3, wp::float32> var_750;
    wp::vec_t<3, wp::float32> var_751;
    wp::float32 var_752;
    wp::float32 var_753;
    const wp::float32 var_754 = 0.5;
    wp::float32 var_755;
    wp::vec_t<3, wp::float32> var_756;
    wp::vec_t<3, wp::float32> var_757;
    const wp::int32 var_758 = 1;
    wp::int32 var_759;
    wp::int32 var_760;
    wp::float32 var_761;
    wp::float32 var_762;
    wp::vec_t<3, wp::float32> var_763;
    //---------
    // forward
    // def plane_convex(plane_normal: wp.vec3, plane_pos: wp.vec3, convex: Geom) -> Tuple[wp.vec4, mat43, wp.vec3]:       <L 52>
    // _HUGE_VAL = 1e6                                                                        <L 65>
    // contact_dist = wp.vec4(MJ_MAXVAL)                                                      <L 67>
    var_2 = wp::vec_t<4, wp::float32>(var_1);
    // contact_pos = mat43()                                                                  <L 68>
    var_3 = wp::mat_t<4, 3, wp::float32>();
    // contact_count = int(0)                                                                 <L 69>
    var_5 = wp::int(var_4);
    // plane_pos_local = wp.transpose(convex.rot) @ (plane_pos - convex.pos)                  <L 72>
    var_6 = &((var_convex).rot);
    var_8 = wp::load(var_6);
    var_7 = wp::transpose(var_8);
    var_9 = &((var_convex).pos);
    var_11 = wp::load(var_9);
    var_10 = wp::sub(var_plane_pos, var_11);
    var_12 = wp::mul(var_7, var_10);
    // n = wp.transpose(convex.rot) @ plane_normal                                            <L 73>
    var_13 = &((var_convex).rot);
    var_15 = wp::load(var_13);
    var_14 = wp::transpose(var_15);
    var_16 = wp::mul(var_14, var_plane_normal);
    // indices = wp.vec4i(-1, -1, -1, -1)                                                     <L 76>
    var_21 = wp::vec_t<4, wp::int32>(var_17, var_18, var_19, var_20);
    // if convex.graphadr == -1 or convex.vertnum < 10:                                       <L 79>
    var_23 = &((var_convex).graphadr);
    var_26 = wp::load(var_23);
    var_25 = (var_26 == var_24);
    var_22 = var_25;
    if (!var_22) {
        var_27 = &((var_convex).vertnum);
        var_30 = wp::load(var_27);
        var_29 = (var_30 < var_28);
        var_22 = var_22 || var_29;
    }
    if (var_22) {
        // max_support = wp.float32(-_HUGE_VAL)                                               <L 81>
        var_32 = wp::float32(var_31);
        // a = wp.vec3()                                                                      <L 82>
        var_33 = wp::vec_t<3, wp::float32>();
        // for i in range(convex.vertnum):                                                    <L 83>
        var_34 = &((var_convex).vertnum);
        var_36 = wp::load(var_34);
        var_35 = wp::range(var_36);
        start_for_0:;
            if (iter_cmp(var_35) == 0) goto end_for_0;
            var_37 = wp::iter_next(var_35);
            // vert = convex.vert[convex.vertadr + i]                                         <L 84>
            var_38 = &((var_convex).vert);
            var_39 = &((var_convex).vertadr);
            var_41 = wp::load(var_39);
            var_40 = wp::add(var_41, var_37);
            var_43 = wp::load(var_38);
            var_42 = wp::address(var_43, var_40);
            var_45 = wp::load(var_42);
            var_44 = wp::copy(var_45);
            // support = wp.dot(plane_pos_local - vert, n)                                    <L 85>
            var_46 = wp::sub(var_12, var_44);
            var_47 = wp::dot(var_46, var_16);
            // if support > max_support:                                                      <L 86>
            var_48 = (var_47 > var_32);
            if (var_48) {
                // max_support = support                                                      <L 87>
                var_49 = wp::copy(var_47);
                // indices[0] = i                                                             <L 88>
                wp::assign_inplace(var_21, var_50, var_37);
                // a = vert                                                                   <L 89>
                var_51 = wp::copy(var_44);
            }
            var_52 = wp::where(var_48, var_49, var_32);
            var_53 = wp::where(var_48, var_51, var_33);
            wp::assign(var_32, var_52);
            wp::assign(var_33, var_53);
            goto start_for_0;
        end_for_0:;
        // if max_support < 0:                                                                <L 91>
        var_55 = (var_32 < var_54);
        if (var_55) {
            // return contact_dist, contact_pos, plane_normal                                 <L 92>
            ret_0 = var_2;
            ret_1 = var_3;
            ret_2 = var_plane_normal;
            return;
        }
        // threshold = max_support - 1e-3                                                     <L 94>
        var_57 = wp::sub(var_32, var_56);
        // b_dist = wp.float32(-_HUGE_VAL)                                                    <L 97>
        var_59 = wp::float32(var_58);
        // b = wp.vec3()                                                                      <L 98>
        var_60 = wp::vec_t<3, wp::float32>();
        // for i in range(convex.vertnum):                                                    <L 99>
        var_61 = &((var_convex).vertnum);
        var_63 = wp::load(var_61);
        var_62 = wp::range(var_63);
        start_for_3:;
            if (iter_cmp(var_62) == 0) goto end_for_3;
            var_64 = wp::iter_next(var_62);
            // vert = convex.vert[convex.vertadr + i]                                         <L 100>
            var_65 = &((var_convex).vert);
            var_66 = &((var_convex).vertadr);
            var_68 = wp::load(var_66);
            var_67 = wp::add(var_68, var_64);
            var_70 = wp::load(var_65);
            var_69 = wp::address(var_70, var_67);
            var_72 = wp::load(var_69);
            var_71 = wp::copy(var_72);
            // support = wp.dot(plane_pos_local - vert, n)                                    <L 101>
            var_73 = wp::sub(var_12, var_71);
            var_74 = wp::dot(var_73, var_16);
            // dist_mask = wp.where(support > threshold, 0.0, -_HUGE_VAL)                     <L 102>
            var_75 = (var_74 > var_57);
            var_78 = wp::where(var_75, var_76, var_77);
            // dist = wp.length_sq(a - vert) + dist_mask                                      <L 103>
            var_79 = wp::sub(var_33, var_71);
            var_80 = wp::length_sq(var_79);
            var_81 = wp::add(var_80, var_78);
            // if dist > b_dist:                                                              <L 104>
            var_82 = (var_81 > var_59);
            if (var_82) {
                // indices[1] = i                                                             <L 105>
                wp::assign_inplace(var_21, var_83, var_64);
                // b_dist = dist                                                              <L 106>
                var_84 = wp::copy(var_81);
                // b = vert                                                                   <L 107>
                var_85 = wp::copy(var_71);
            }
            var_86 = wp::where(var_82, var_84, var_59);
            var_87 = wp::where(var_82, var_85, var_60);
            wp::assign(var_44, var_71);
            wp::assign(var_47, var_74);
            wp::assign(var_59, var_86);
            wp::assign(var_60, var_87);
            goto start_for_3;
        end_for_3:;
        // ab = wp.cross(n, a - b)                                                            <L 110>
        var_88 = wp::sub(var_33, var_60);
        var_89 = wp::cross(var_16, var_88);
        // c_dist = wp.float32(-_HUGE_VAL)                                                    <L 111>
        var_91 = wp::float32(var_90);
        // c = wp.vec3()                                                                      <L 112>
        var_92 = wp::vec_t<3, wp::float32>();
        // for i in range(convex.vertnum):                                                    <L 113>
        var_93 = &((var_convex).vertnum);
        var_95 = wp::load(var_93);
        var_94 = wp::range(var_95);
        start_for_5:;
            if (iter_cmp(var_94) == 0) goto end_for_5;
            var_96 = wp::iter_next(var_94);
            // vert = convex.vert[convex.vertadr + i]                                         <L 114>
            var_97 = &((var_convex).vert);
            var_98 = &((var_convex).vertadr);
            var_100 = wp::load(var_98);
            var_99 = wp::add(var_100, var_96);
            var_102 = wp::load(var_97);
            var_101 = wp::address(var_102, var_99);
            var_104 = wp::load(var_101);
            var_103 = wp::copy(var_104);
            // support = wp.dot(plane_pos_local - vert, n)                                    <L 115>
            var_105 = wp::sub(var_12, var_103);
            var_106 = wp::dot(var_105, var_16);
            // dist_mask = wp.where(support > threshold, 0.0, -_HUGE_VAL)                     <L 116>
            var_107 = (var_106 > var_57);
            var_110 = wp::where(var_107, var_108, var_109);
            // ap = a - vert                                                                  <L 117>
            var_111 = wp::sub(var_33, var_103);
            // dist = wp.abs(wp.dot(ap, ab)) + dist_mask                                      <L 118>
            var_112 = wp::dot(var_111, var_89);
            var_113 = wp::abs(var_112);
            var_114 = wp::add(var_113, var_110);
            // if dist > c_dist:                                                              <L 119>
            var_115 = (var_114 > var_91);
            if (var_115) {
                // indices[2] = i                                                             <L 120>
                wp::assign_inplace(var_21, var_116, var_96);
                // c_dist = dist                                                              <L 121>
                var_117 = wp::copy(var_114);
                // c = vert                                                                   <L 122>
                var_118 = wp::copy(var_103);
            }
            var_119 = wp::where(var_115, var_117, var_91);
            var_120 = wp::where(var_115, var_118, var_92);
            wp::assign(var_44, var_103);
            wp::assign(var_47, var_106);
            wp::assign(var_78, var_110);
            wp::assign(var_81, var_114);
            wp::assign(var_91, var_119);
            wp::assign(var_92, var_120);
            goto start_for_5;
        end_for_5:;
        // ac = wp.cross(n, a - c)                                                            <L 125>
        var_121 = wp::sub(var_33, var_92);
        var_122 = wp::cross(var_16, var_121);
        // bc = wp.cross(n, b - c)                                                            <L 126>
        var_123 = wp::sub(var_60, var_92);
        var_124 = wp::cross(var_16, var_123);
        // d_dist = wp.float32(-_HUGE_VAL)                                                    <L 127>
        var_126 = wp::float32(var_125);
        // for i in range(convex.vertnum):                                                    <L 128>
        var_127 = &((var_convex).vertnum);
        var_129 = wp::load(var_127);
        var_128 = wp::range(var_129);
        start_for_7:;
            if (iter_cmp(var_128) == 0) goto end_for_7;
            var_130 = wp::iter_next(var_128);
            // vert = convex.vert[convex.vertadr + i]                                         <L 129>
            var_131 = &((var_convex).vert);
            var_132 = &((var_convex).vertadr);
            var_134 = wp::load(var_132);
            var_133 = wp::add(var_134, var_130);
            var_136 = wp::load(var_131);
            var_135 = wp::address(var_136, var_133);
            var_138 = wp::load(var_135);
            var_137 = wp::copy(var_138);
            // support = wp.dot(plane_pos_local - vert, n)                                    <L 130>
            var_139 = wp::sub(var_12, var_137);
            var_140 = wp::dot(var_139, var_16);
            // dist_mask = wp.where(support > threshold, 0.0, -_HUGE_VAL)                     <L 131>
            var_141 = (var_140 > var_57);
            var_144 = wp::where(var_141, var_142, var_143);
            // ap = a - vert                                                                  <L 132>
            var_145 = wp::sub(var_33, var_137);
            // bp = b - vert                                                                  <L 133>
            var_146 = wp::sub(var_60, var_137);
            // dist_ap = wp.abs(wp.dot(ap, ac)) + dist_mask                                   <L 134>
            var_147 = wp::dot(var_145, var_122);
            var_148 = wp::abs(var_147);
            var_149 = wp::add(var_148, var_144);
            // dist_bp = wp.abs(wp.dot(bp, bc)) + dist_mask                                   <L 135>
            var_150 = wp::dot(var_146, var_124);
            var_151 = wp::abs(var_150);
            var_152 = wp::add(var_151, var_144);
            // if dist_ap + dist_bp > d_dist:                                                 <L 136>
            var_153 = wp::add(var_149, var_152);
            var_154 = (var_153 > var_126);
            if (var_154) {
                // indices[3] = i                                                             <L 137>
                wp::assign_inplace(var_21, var_155, var_130);
                // d_dist = dist_ap + dist_bp                                                 <L 138>
                var_156 = wp::add(var_149, var_152);
            }
            var_157 = wp::where(var_154, var_156, var_126);
            wp::assign(var_44, var_137);
            wp::assign(var_47, var_140);
            wp::assign(var_78, var_144);
            wp::assign(var_111, var_145);
            wp::assign(var_126, var_157);
            goto start_for_7;
        end_for_7:;
    }
    if (!var_22) {
        // numvert = convex.graph[convex.graphadr]                                            <L 141>
        var_158 = &((var_convex).graph);
        var_159 = &((var_convex).graphadr);
        var_160 = wp::load(var_159);
        var_162 = wp::load(var_158);
        var_161 = wp::address(var_162, var_160);
        var_164 = wp::load(var_161);
        var_163 = wp::copy(var_164);
        // vert_edgeadr = convex.graphadr + 2                                                 <L 142>
        var_165 = &((var_convex).graphadr);
        var_168 = wp::load(var_165);
        var_167 = wp::add(var_168, var_166);
        // vert_globalid = convex.graphadr + 2 + numvert                                      <L 143>
        var_169 = &((var_convex).graphadr);
        var_172 = wp::load(var_169);
        var_171 = wp::add(var_172, var_170);
        var_173 = wp::add(var_171, var_163);
        // edge_localid = convex.graphadr + 2 + 2 * numvert                                   <L 144>
        var_174 = &((var_convex).graphadr);
        var_177 = wp::load(var_174);
        var_176 = wp::add(var_177, var_175);
        var_179 = wp::mul(var_178, var_163);
        var_180 = wp::add(var_176, var_179);
        // max_support = wp.float32(-_HUGE_VAL)                                               <L 147>
        var_182 = wp::float32(var_181);
        // prev = int(-1)                                                                     <L 150>
        var_184 = wp::int(var_183);
        // imax = int(0)                                                                      <L 151>
        var_186 = wp::int(var_185);
        // while True:                                                                        <L 153>
    start_while_9:;
    if ((var_187) == false) goto end_while_9;
            // prev = int(imax)                                                               <L 154>
            var_188 = wp::int(var_186);
            // i = int(convex.graph[vert_edgeadr + imax])                                     <L 155>
            var_189 = &((var_convex).graph);
            var_190 = wp::add(var_167, var_186);
            var_192 = wp::load(var_189);
            var_191 = wp::address(var_192, var_190);
            var_194 = wp::load(var_191);
            var_193 = wp::int(var_194);
            // while convex.graph[edge_localid + i] >= 0:                                     <L 156>
    start_while_11:;
            var_195 = &((var_convex).graph);
            var_196 = wp::add(var_180, var_193);
            var_198 = wp::load(var_195);
            var_197 = wp::address(var_198, var_196);
            var_201 = wp::load(var_197);
            var_200 = (var_201 >= var_199);
    if ((var_200) == false) goto end_while_11;
                // subidx = convex.graph[edge_localid + i]                                    <L 157>
                var_202 = &((var_convex).graph);
                var_203 = wp::add(var_180, var_193);
                var_205 = wp::load(var_202);
                var_204 = wp::address(var_205, var_203);
                var_207 = wp::load(var_204);
                var_206 = wp::copy(var_207);
                // idx = convex.graph[vert_globalid + subidx]                                 <L 158>
                var_208 = &((var_convex).graph);
                var_209 = wp::add(var_173, var_206);
                var_211 = wp::load(var_208);
                var_210 = wp::address(var_211, var_209);
                var_213 = wp::load(var_210);
                var_212 = wp::copy(var_213);
                // support = wp.dot(plane_pos_local - convex.vert[convex.vertadr + idx], n)       <L 159>
                var_214 = &((var_convex).vert);
                var_215 = &((var_convex).vertadr);
                var_217 = wp::load(var_215);
                var_216 = wp::add(var_217, var_212);
                var_219 = wp::load(var_214);
                var_218 = wp::address(var_219, var_216);
                var_221 = wp::load(var_218);
                var_220 = wp::sub(var_12, var_221);
                var_222 = wp::dot(var_220, var_16);
                // if support > max_support:                                                  <L 160>
                var_223 = (var_222 > var_182);
                if (var_223) {
                    // max_support = support                                                  <L 161>
                    var_224 = wp::copy(var_222);
                    // imax = int(subidx)                                                     <L 162>
                    var_225 = wp::int(var_206);
                }
                var_226 = wp::where(var_223, var_224, var_182);
                var_227 = wp::where(var_223, var_225, var_186);
                // i += int(1)                                                                <L 163>
                var_229 = wp::int(var_228);
                var_230 = wp::add(var_193, var_229);
                wp::assign(var_182, var_226);
                wp::assign(var_186, var_227);
                wp::assign(var_193, var_230);
    goto start_while_11;
    end_while_11:;
            // if imax == prev:                                                               <L 164>
            var_231 = (var_186 == var_188);
            if (var_231) {
                // break                                                                      <L 165>
                wp::assign(var_184, var_188);
                goto end_while_9;
            }
            var_232 = wp::where(var_231, var_184, var_188);
            wp::assign(var_184, var_232);
    goto start_while_9;
    end_while_9:;
        // threshold = wp.max(0.0, max_support - 1e-3)                                        <L 167>
        var_235 = wp::sub(var_182, var_234);
        var_236 = wp::max(var_233, var_235);
        // a_dist = wp.float32(-_HUGE_VAL)                                                    <L 169>
        var_238 = wp::float32(var_237);
        // while True:                                                                        <L 170>
    start_while_13:;
    if ((var_239) == false) goto end_while_13;
            // prev = int(imax)                                                               <L 171>
            var_240 = wp::int(var_186);
            // i = int(convex.graph[vert_edgeadr + imax])                                     <L 172>
            var_241 = &((var_convex).graph);
            var_242 = wp::add(var_167, var_186);
            var_244 = wp::load(var_241);
            var_243 = wp::address(var_244, var_242);
            var_246 = wp::load(var_243);
            var_245 = wp::int(var_246);
            // while convex.graph[edge_localid + i] >= 0:                                     <L 173>
    start_while_15:;
            var_247 = &((var_convex).graph);
            var_248 = wp::add(var_180, var_245);
            var_250 = wp::load(var_247);
            var_249 = wp::address(var_250, var_248);
            var_253 = wp::load(var_249);
            var_252 = (var_253 >= var_251);
    if ((var_252) == false) goto end_while_15;
                // subidx = convex.graph[edge_localid + i]                                    <L 174>
                var_254 = &((var_convex).graph);
                var_255 = wp::add(var_180, var_245);
                var_257 = wp::load(var_254);
                var_256 = wp::address(var_257, var_255);
                var_259 = wp::load(var_256);
                var_258 = wp::copy(var_259);
                // idx = convex.graph[vert_globalid + subidx]                                 <L 175>
                var_260 = &((var_convex).graph);
                var_261 = wp::add(var_173, var_258);
                var_263 = wp::load(var_260);
                var_262 = wp::address(var_263, var_261);
                var_265 = wp::load(var_262);
                var_264 = wp::copy(var_265);
                // support = wp.dot(plane_pos_local - convex.vert[convex.vertadr + idx], n)       <L 176>
                var_266 = &((var_convex).vert);
                var_267 = &((var_convex).vertadr);
                var_269 = wp::load(var_267);
                var_268 = wp::add(var_269, var_264);
                var_271 = wp::load(var_266);
                var_270 = wp::address(var_271, var_268);
                var_273 = wp::load(var_270);
                var_272 = wp::sub(var_12, var_273);
                var_274 = wp::dot(var_272, var_16);
                // dist = wp.where(support > threshold, support, -_HUGE_VAL)                  <L 177>
                var_275 = (var_274 > var_236);
                var_277 = wp::where(var_275, var_274, var_276);
                // if dist > a_dist:                                                          <L 178>
                var_278 = (var_277 > var_238);
                if (var_278) {
                    // a_dist = dist                                                          <L 179>
                    var_279 = wp::copy(var_277);
                    // imax = int(subidx)                                                     <L 180>
                    var_280 = wp::int(var_258);
                }
                var_281 = wp::where(var_278, var_280, var_186);
                var_282 = wp::where(var_278, var_279, var_238);
                // i += int(1)                                                                <L 181>
                var_284 = wp::int(var_283);
                var_285 = wp::add(var_245, var_284);
                wp::assign(var_186, var_281);
                wp::assign(var_245, var_285);
                wp::assign(var_206, var_258);
                wp::assign(var_212, var_264);
                wp::assign(var_222, var_274);
                wp::assign(var_238, var_282);
    goto start_while_15;
    end_while_15:;
            // if imax == prev:                                                               <L 182>
            var_286 = (var_186 == var_240);
            if (var_286) {
                // break                                                                      <L 183>
                wp::assign(var_184, var_240);
                wp::assign(var_193, var_245);
                goto end_while_13;
            }
            var_287 = wp::where(var_286, var_184, var_240);
            var_288 = wp::where(var_286, var_193, var_245);
            wp::assign(var_184, var_287);
            wp::assign(var_193, var_288);
    goto start_while_13;
    end_while_13:;
        // imax_global = convex.graph[vert_globalid + imax]                                   <L 184>
        var_289 = &((var_convex).graph);
        var_290 = wp::add(var_173, var_186);
        var_292 = wp::load(var_289);
        var_291 = wp::address(var_292, var_290);
        var_294 = wp::load(var_291);
        var_293 = wp::copy(var_294);
        // a = convex.vert[convex.vertadr + imax_global]                                      <L 185>
        var_295 = &((var_convex).vert);
        var_296 = &((var_convex).vertadr);
        var_298 = wp::load(var_296);
        var_297 = wp::add(var_298, var_293);
        var_300 = wp::load(var_295);
        var_299 = wp::address(var_300, var_297);
        var_302 = wp::load(var_299);
        var_301 = wp::copy(var_302);
        // indices[0] = imax_global                                                           <L 186>
        wp::assign_inplace(var_21, var_303, var_293);
        // b_dist = wp.float32(-_HUGE_VAL)                                                    <L 189>
        var_305 = wp::float32(var_304);
        // while True:                                                                        <L 190>
    start_while_17:;
    if ((var_306) == false) goto end_while_17;
            // prev = int(imax)                                                               <L 191>
            var_307 = wp::int(var_186);
            // i = int(convex.graph[vert_edgeadr + imax])                                     <L 192>
            var_308 = &((var_convex).graph);
            var_309 = wp::add(var_167, var_186);
            var_311 = wp::load(var_308);
            var_310 = wp::address(var_311, var_309);
            var_313 = wp::load(var_310);
            var_312 = wp::int(var_313);
            // while convex.graph[edge_localid + i] >= 0:                                     <L 193>
    start_while_19:;
            var_314 = &((var_convex).graph);
            var_315 = wp::add(var_180, var_312);
            var_317 = wp::load(var_314);
            var_316 = wp::address(var_317, var_315);
            var_320 = wp::load(var_316);
            var_319 = (var_320 >= var_318);
    if ((var_319) == false) goto end_while_19;
                // subidx = convex.graph[edge_localid + i]                                    <L 194>
                var_321 = &((var_convex).graph);
                var_322 = wp::add(var_180, var_312);
                var_324 = wp::load(var_321);
                var_323 = wp::address(var_324, var_322);
                var_326 = wp::load(var_323);
                var_325 = wp::copy(var_326);
                // idx = convex.graph[vert_globalid + subidx]                                 <L 195>
                var_327 = &((var_convex).graph);
                var_328 = wp::add(var_173, var_325);
                var_330 = wp::load(var_327);
                var_329 = wp::address(var_330, var_328);
                var_332 = wp::load(var_329);
                var_331 = wp::copy(var_332);
                // support = wp.dot(plane_pos_local - convex.vert[convex.vertadr + idx], n)       <L 196>
                var_333 = &((var_convex).vert);
                var_334 = &((var_convex).vertadr);
                var_336 = wp::load(var_334);
                var_335 = wp::add(var_336, var_331);
                var_338 = wp::load(var_333);
                var_337 = wp::address(var_338, var_335);
                var_340 = wp::load(var_337);
                var_339 = wp::sub(var_12, var_340);
                var_341 = wp::dot(var_339, var_16);
                // dist_mask = wp.where(support > threshold, 0.0, -_HUGE_VAL)                 <L 197>
                var_342 = (var_341 > var_236);
                var_345 = wp::where(var_342, var_343, var_344);
                // dist = wp.length_sq(a - convex.vert[convex.vertadr + idx]) + dist_mask       <L 198>
                var_346 = &((var_convex).vert);
                var_347 = &((var_convex).vertadr);
                var_349 = wp::load(var_347);
                var_348 = wp::add(var_349, var_331);
                var_351 = wp::load(var_346);
                var_350 = wp::address(var_351, var_348);
                var_353 = wp::load(var_350);
                var_352 = wp::sub(var_301, var_353);
                var_354 = wp::length_sq(var_352);
                var_355 = wp::add(var_354, var_345);
                // if dist > b_dist:                                                          <L 199>
                var_356 = (var_355 > var_305);
                if (var_356) {
                    // b_dist = dist                                                          <L 200>
                    var_357 = wp::copy(var_355);
                    // imax = int(subidx)                                                     <L 201>
                    var_358 = wp::int(var_325);
                }
                var_359 = wp::where(var_356, var_358, var_186);
                var_360 = wp::where(var_356, var_357, var_305);
                // i += int(1)                                                                <L 202>
                var_362 = wp::int(var_361);
                var_363 = wp::add(var_312, var_362);
                wp::assign(var_186, var_359);
                wp::assign(var_312, var_363);
                wp::assign(var_206, var_325);
                wp::assign(var_212, var_331);
                wp::assign(var_222, var_341);
                wp::assign(var_277, var_355);
                wp::assign(var_305, var_360);
    goto start_while_19;
    end_while_19:;
            // if imax == prev:                                                               <L 203>
            var_364 = (var_186 == var_307);
            if (var_364) {
                // break                                                                      <L 204>
                wp::assign(var_184, var_307);
                wp::assign(var_193, var_312);
                goto end_while_17;
            }
            var_365 = wp::where(var_364, var_184, var_307);
            var_366 = wp::where(var_364, var_193, var_312);
            wp::assign(var_184, var_365);
            wp::assign(var_193, var_366);
    goto start_while_17;
    end_while_17:;
        // imax_global = convex.graph[vert_globalid + imax]                                   <L 205>
        var_367 = &((var_convex).graph);
        var_368 = wp::add(var_173, var_186);
        var_370 = wp::load(var_367);
        var_369 = wp::address(var_370, var_368);
        var_372 = wp::load(var_369);
        var_371 = wp::copy(var_372);
        // b = convex.vert[convex.vertadr + imax_global]                                      <L 206>
        var_373 = &((var_convex).vert);
        var_374 = &((var_convex).vertadr);
        var_376 = wp::load(var_374);
        var_375 = wp::add(var_376, var_371);
        var_378 = wp::load(var_373);
        var_377 = wp::address(var_378, var_375);
        var_380 = wp::load(var_377);
        var_379 = wp::copy(var_380);
        // indices[1] = imax_global                                                           <L 207>
        wp::assign_inplace(var_21, var_381, var_371);
        // ab = wp.cross(n, a - b)                                                            <L 210>
        var_382 = wp::sub(var_301, var_379);
        var_383 = wp::cross(var_16, var_382);
        // c_dist = wp.float32(-_HUGE_VAL)                                                    <L 211>
        var_385 = wp::float32(var_384);
        // while True:                                                                        <L 212>
    start_while_21:;
    if ((var_386) == false) goto end_while_21;
            // prev = int(imax)                                                               <L 213>
            var_387 = wp::int(var_186);
            // i = int(convex.graph[vert_edgeadr + imax])                                     <L 214>
            var_388 = &((var_convex).graph);
            var_389 = wp::add(var_167, var_186);
            var_391 = wp::load(var_388);
            var_390 = wp::address(var_391, var_389);
            var_393 = wp::load(var_390);
            var_392 = wp::int(var_393);
            // while convex.graph[edge_localid + i] >= 0:                                     <L 215>
    start_while_23:;
            var_394 = &((var_convex).graph);
            var_395 = wp::add(var_180, var_392);
            var_397 = wp::load(var_394);
            var_396 = wp::address(var_397, var_395);
            var_400 = wp::load(var_396);
            var_399 = (var_400 >= var_398);
    if ((var_399) == false) goto end_while_23;
                // subidx = convex.graph[edge_localid + i]                                    <L 216>
                var_401 = &((var_convex).graph);
                var_402 = wp::add(var_180, var_392);
                var_404 = wp::load(var_401);
                var_403 = wp::address(var_404, var_402);
                var_406 = wp::load(var_403);
                var_405 = wp::copy(var_406);
                // idx = convex.graph[vert_globalid + subidx]                                 <L 217>
                var_407 = &((var_convex).graph);
                var_408 = wp::add(var_173, var_405);
                var_410 = wp::load(var_407);
                var_409 = wp::address(var_410, var_408);
                var_412 = wp::load(var_409);
                var_411 = wp::copy(var_412);
                // support = wp.dot(plane_pos_local - convex.vert[convex.vertadr + idx], n)       <L 218>
                var_413 = &((var_convex).vert);
                var_414 = &((var_convex).vertadr);
                var_416 = wp::load(var_414);
                var_415 = wp::add(var_416, var_411);
                var_418 = wp::load(var_413);
                var_417 = wp::address(var_418, var_415);
                var_420 = wp::load(var_417);
                var_419 = wp::sub(var_12, var_420);
                var_421 = wp::dot(var_419, var_16);
                // dist_mask = wp.where(support > threshold, 0.0, -_HUGE_VAL)                 <L 219>
                var_422 = (var_421 > var_236);
                var_425 = wp::where(var_422, var_423, var_424);
                // ap = a - convex.vert[convex.vertadr + idx]                                 <L 220>
                var_426 = &((var_convex).vert);
                var_427 = &((var_convex).vertadr);
                var_429 = wp::load(var_427);
                var_428 = wp::add(var_429, var_411);
                var_431 = wp::load(var_426);
                var_430 = wp::address(var_431, var_428);
                var_433 = wp::load(var_430);
                var_432 = wp::sub(var_301, var_433);
                // dist = wp.abs(wp.dot(ap, ab)) + dist_mask                                  <L 221>
                var_434 = wp::dot(var_432, var_383);
                var_435 = wp::abs(var_434);
                var_436 = wp::add(var_435, var_425);
                // if dist > c_dist:                                                          <L 222>
                var_437 = (var_436 > var_385);
                if (var_437) {
                    // c_dist = dist                                                          <L 223>
                    var_438 = wp::copy(var_436);
                    // imax = int(subidx)                                                     <L 224>
                    var_439 = wp::int(var_405);
                }
                var_440 = wp::where(var_437, var_439, var_186);
                var_441 = wp::where(var_437, var_438, var_385);
                // i += int(1)                                                                <L 225>
                var_443 = wp::int(var_442);
                var_444 = wp::add(var_392, var_443);
                wp::assign(var_186, var_440);
                wp::assign(var_392, var_444);
                wp::assign(var_206, var_405);
                wp::assign(var_212, var_411);
                wp::assign(var_222, var_421);
                wp::assign(var_277, var_436);
                wp::assign(var_345, var_425);
                wp::assign(var_385, var_441);
    goto start_while_23;
    end_while_23:;
            // if imax == prev:                                                               <L 226>
            var_445 = (var_186 == var_387);
            if (var_445) {
                // break                                                                      <L 227>
                wp::assign(var_184, var_387);
                wp::assign(var_193, var_392);
                goto end_while_21;
            }
            var_446 = wp::where(var_445, var_184, var_387);
            var_447 = wp::where(var_445, var_193, var_392);
            wp::assign(var_184, var_446);
            wp::assign(var_193, var_447);
    goto start_while_21;
    end_while_21:;
        // imax_global = convex.graph[vert_globalid + imax]                                   <L 228>
        var_448 = &((var_convex).graph);
        var_449 = wp::add(var_173, var_186);
        var_451 = wp::load(var_448);
        var_450 = wp::address(var_451, var_449);
        var_453 = wp::load(var_450);
        var_452 = wp::copy(var_453);
        // c = convex.vert[convex.vertadr + imax_global]                                      <L 229>
        var_454 = &((var_convex).vert);
        var_455 = &((var_convex).vertadr);
        var_457 = wp::load(var_455);
        var_456 = wp::add(var_457, var_452);
        var_459 = wp::load(var_454);
        var_458 = wp::address(var_459, var_456);
        var_461 = wp::load(var_458);
        var_460 = wp::copy(var_461);
        // indices[2] = imax_global                                                           <L 230>
        wp::assign_inplace(var_21, var_462, var_452);
        // ac = wp.cross(n, a - c)                                                            <L 233>
        var_463 = wp::sub(var_301, var_460);
        var_464 = wp::cross(var_16, var_463);
        // bc = wp.cross(n, b - c)                                                            <L 234>
        var_465 = wp::sub(var_379, var_460);
        var_466 = wp::cross(var_16, var_465);
        // d_dist = wp.float32(-_HUGE_VAL)                                                    <L 235>
        var_468 = wp::float32(var_467);
        // while True:                                                                        <L 236>
    start_while_25:;
    if ((var_469) == false) goto end_while_25;
            // prev = int(imax)                                                               <L 237>
            var_470 = wp::int(var_186);
            // i = int(convex.graph[vert_edgeadr + imax])                                     <L 238>
            var_471 = &((var_convex).graph);
            var_472 = wp::add(var_167, var_186);
            var_474 = wp::load(var_471);
            var_473 = wp::address(var_474, var_472);
            var_476 = wp::load(var_473);
            var_475 = wp::int(var_476);
            // while convex.graph[edge_localid + i] >= 0:                                     <L 239>
    start_while_27:;
            var_477 = &((var_convex).graph);
            var_478 = wp::add(var_180, var_475);
            var_480 = wp::load(var_477);
            var_479 = wp::address(var_480, var_478);
            var_483 = wp::load(var_479);
            var_482 = (var_483 >= var_481);
    if ((var_482) == false) goto end_while_27;
                // subidx = convex.graph[edge_localid + i]                                    <L 240>
                var_484 = &((var_convex).graph);
                var_485 = wp::add(var_180, var_475);
                var_487 = wp::load(var_484);
                var_486 = wp::address(var_487, var_485);
                var_489 = wp::load(var_486);
                var_488 = wp::copy(var_489);
                // idx = convex.graph[vert_globalid + subidx]                                 <L 241>
                var_490 = &((var_convex).graph);
                var_491 = wp::add(var_173, var_488);
                var_493 = wp::load(var_490);
                var_492 = wp::address(var_493, var_491);
                var_495 = wp::load(var_492);
                var_494 = wp::copy(var_495);
                // support = wp.dot(plane_pos_local - convex.vert[convex.vertadr + idx], n)       <L 242>
                var_496 = &((var_convex).vert);
                var_497 = &((var_convex).vertadr);
                var_499 = wp::load(var_497);
                var_498 = wp::add(var_499, var_494);
                var_501 = wp::load(var_496);
                var_500 = wp::address(var_501, var_498);
                var_503 = wp::load(var_500);
                var_502 = wp::sub(var_12, var_503);
                var_504 = wp::dot(var_502, var_16);
                // dist_mask = wp.where(support > threshold, 0.0, -_HUGE_VAL)                 <L 243>
                var_505 = (var_504 > var_236);
                var_508 = wp::where(var_505, var_506, var_507);
                // ap = a - convex.vert[convex.vertadr + idx]                                 <L 244>
                var_509 = &((var_convex).vert);
                var_510 = &((var_convex).vertadr);
                var_512 = wp::load(var_510);
                var_511 = wp::add(var_512, var_494);
                var_514 = wp::load(var_509);
                var_513 = wp::address(var_514, var_511);
                var_516 = wp::load(var_513);
                var_515 = wp::sub(var_301, var_516);
                // bp = b - convex.vert[convex.vertadr + idx]                                 <L 245>
                var_517 = &((var_convex).vert);
                var_518 = &((var_convex).vertadr);
                var_520 = wp::load(var_518);
                var_519 = wp::add(var_520, var_494);
                var_522 = wp::load(var_517);
                var_521 = wp::address(var_522, var_519);
                var_524 = wp::load(var_521);
                var_523 = wp::sub(var_379, var_524);
                // dist_ap = wp.abs(wp.dot(ap, ac)) + dist_mask                               <L 246>
                var_525 = wp::dot(var_515, var_464);
                var_526 = wp::abs(var_525);
                var_527 = wp::add(var_526, var_508);
                // dist_bp = wp.abs(wp.dot(bp, bc)) + dist_mask                               <L 247>
                var_528 = wp::dot(var_523, var_466);
                var_529 = wp::abs(var_528);
                var_530 = wp::add(var_529, var_508);
                // if dist_ap + dist_bp > d_dist:                                             <L 248>
                var_531 = wp::add(var_527, var_530);
                var_532 = (var_531 > var_468);
                if (var_532) {
                    // d_dist = dist_ap + dist_bp                                             <L 249>
                    var_533 = wp::add(var_527, var_530);
                    // imax = int(subidx)                                                     <L 250>
                    var_534 = wp::int(var_488);
                }
                var_535 = wp::where(var_532, var_534, var_186);
                var_536 = wp::where(var_532, var_533, var_468);
                // i += int(1)                                                                <L 251>
                var_538 = wp::int(var_537);
                var_539 = wp::add(var_475, var_538);
                wp::assign(var_186, var_535);
                wp::assign(var_475, var_539);
                wp::assign(var_206, var_488);
                wp::assign(var_212, var_494);
                wp::assign(var_222, var_504);
                wp::assign(var_345, var_508);
                wp::assign(var_432, var_515);
                wp::assign(var_468, var_536);
    goto start_while_27;
    end_while_27:;
            // if imax == prev:                                                               <L 252>
            var_540 = (var_186 == var_470);
            if (var_540) {
                // break                                                                      <L 253>
                wp::assign(var_184, var_470);
                wp::assign(var_193, var_475);
                goto end_while_25;
            }
            var_541 = wp::where(var_540, var_184, var_470);
            var_542 = wp::where(var_540, var_193, var_475);
            wp::assign(var_184, var_541);
            wp::assign(var_193, var_542);
    goto start_while_25;
    end_while_25:;
        // imax_global = convex.graph[vert_globalid + imax]                                   <L 254>
        var_543 = &((var_convex).graph);
        var_544 = wp::add(var_173, var_186);
        var_546 = wp::load(var_543);
        var_545 = wp::address(var_546, var_544);
        var_548 = wp::load(var_545);
        var_547 = wp::copy(var_548);
        // indices[3] = imax_global                                                           <L 255>
        wp::assign_inplace(var_21, var_549, var_547);
    }
    var_550 = wp::where(var_22, var_32, var_182);
    var_551 = wp::where(var_22, var_33, var_301);
    var_552 = wp::where(var_22, var_130, var_193);
    var_553 = wp::where(var_22, var_47, var_222);
    var_554 = wp::where(var_22, var_57, var_236);
    var_555 = wp::where(var_22, var_59, var_305);
    var_556 = wp::where(var_22, var_60, var_379);
    var_557 = wp::where(var_22, var_78, var_345);
    var_558 = wp::where(var_22, var_81, var_277);
    var_559 = wp::where(var_22, var_89, var_383);
    var_560 = wp::where(var_22, var_91, var_385);
    var_561 = wp::where(var_22, var_92, var_460);
    var_562 = wp::where(var_22, var_111, var_432);
    var_563 = wp::where(var_22, var_122, var_464);
    var_564 = wp::where(var_22, var_124, var_466);
    var_565 = wp::where(var_22, var_126, var_468);
    var_566 = wp::where(var_22, var_146, var_523);
    var_567 = wp::where(var_22, var_149, var_527);
    var_568 = wp::where(var_22, var_152, var_530);
    // for i in range(3, -1, -1):                                                             <L 258>
    // idx = indices[i]                                                                       <L 259>
    var_570 = wp::extract(var_21, var_569);
    // count = int(0)                                                                         <L 260>
    var_572 = wp::int(var_571);
    // for j in range(i + 1):                                                                 <L 261>
    var_574 = wp::add(var_569, var_573);
    var_575 = wp::range(var_574);
    start_for_29:;
        if (iter_cmp(var_575) == 0) goto end_for_29;
        var_576 = wp::iter_next(var_575);
        // if indices[j] == idx:                                                              <L 262>
        var_577 = wp::extract(var_21, var_576);
        var_578 = (var_577 == var_570);
        if (var_578) {
            // count = count + 1                                                              <L 263>
            var_580 = wp::add(var_572, var_579);
        }
        var_581 = wp::where(var_578, var_580, var_572);
        wp::assign(var_572, var_581);
        goto start_for_29;
    end_for_29:;
    // if count == 1:                                                                         <L 266>
    var_583 = (var_572 == var_582);
    if (var_583) {
        // pos = convex.vert[convex.vertadr + idx]                                            <L 267>
        var_584 = &((var_convex).vert);
        var_585 = &((var_convex).vertadr);
        var_587 = wp::load(var_585);
        var_586 = wp::add(var_587, var_570);
        var_589 = wp::load(var_584);
        var_588 = wp::address(var_589, var_586);
        var_591 = wp::load(var_588);
        var_590 = wp::copy(var_591);
        // pos = convex.pos + convex.rot @ pos                                                <L 268>
        var_592 = &((var_convex).pos);
        var_593 = &((var_convex).rot);
        var_595 = wp::load(var_593);
        var_594 = wp::mul(var_595, var_590);
        var_597 = wp::load(var_592);
        var_596 = wp::add(var_597, var_594);
        // support = wp.dot(plane_pos_local - convex.vert[convex.vertadr + idx], n)           <L 269>
        var_598 = &((var_convex).vert);
        var_599 = &((var_convex).vertadr);
        var_601 = wp::load(var_599);
        var_600 = wp::add(var_601, var_570);
        var_603 = wp::load(var_598);
        var_602 = wp::address(var_603, var_600);
        var_605 = wp::load(var_602);
        var_604 = wp::sub(var_12, var_605);
        var_606 = wp::dot(var_604, var_16);
        // dist = -support                                                                    <L 270>
        var_607 = wp::neg(var_606);
        // pos = pos - 0.5 * dist * plane_normal                                              <L 271>
        var_609 = wp::mul(var_608, var_607);
        var_610 = wp::mul(var_609, var_plane_normal);
        var_611 = wp::sub(var_596, var_610);
        // contact_dist[contact_count] = dist                                                 <L 273>
        wp::assign_inplace(var_2, var_5, var_607);
        // contact_pos[contact_count] = pos                                                   <L 274>
        wp::assign_inplace(var_3, var_5, var_611);
        // contact_count = contact_count + 1                                                  <L 275>
        var_613 = wp::add(var_5, var_612);
    }
    var_614 = wp::where(var_583, var_613, var_5);
    var_615 = wp::where(var_583, var_606, var_553);
    var_616 = wp::where(var_583, var_607, var_558);
    // idx = indices[i]                                                                       <L 259>
    var_618 = wp::extract(var_21, var_617);
    // count = int(0)                                                                         <L 260>
    var_620 = wp::int(var_619);
    // for j in range(i + 1):                                                                 <L 261>
    var_622 = wp::add(var_617, var_621);
    var_623 = wp::range(var_622);
    start_for_31:;
        if (iter_cmp(var_623) == 0) goto end_for_31;
        var_624 = wp::iter_next(var_623);
        // if indices[j] == idx:                                                              <L 262>
        var_625 = wp::extract(var_21, var_624);
        var_626 = (var_625 == var_618);
        if (var_626) {
            // count = count + 1                                                              <L 263>
            var_628 = wp::add(var_620, var_627);
        }
        var_629 = wp::where(var_626, var_628, var_620);
        wp::assign(var_620, var_629);
        goto start_for_31;
    end_for_31:;
    // if count == 1:                                                                         <L 266>
    var_631 = (var_620 == var_630);
    if (var_631) {
        // pos = convex.vert[convex.vertadr + idx]                                            <L 267>
        var_632 = &((var_convex).vert);
        var_633 = &((var_convex).vertadr);
        var_635 = wp::load(var_633);
        var_634 = wp::add(var_635, var_618);
        var_637 = wp::load(var_632);
        var_636 = wp::address(var_637, var_634);
        var_639 = wp::load(var_636);
        var_638 = wp::copy(var_639);
        // pos = convex.pos + convex.rot @ pos                                                <L 268>
        var_640 = &((var_convex).pos);
        var_641 = &((var_convex).rot);
        var_643 = wp::load(var_641);
        var_642 = wp::mul(var_643, var_638);
        var_645 = wp::load(var_640);
        var_644 = wp::add(var_645, var_642);
        // support = wp.dot(plane_pos_local - convex.vert[convex.vertadr + idx], n)           <L 269>
        var_646 = &((var_convex).vert);
        var_647 = &((var_convex).vertadr);
        var_649 = wp::load(var_647);
        var_648 = wp::add(var_649, var_618);
        var_651 = wp::load(var_646);
        var_650 = wp::address(var_651, var_648);
        var_653 = wp::load(var_650);
        var_652 = wp::sub(var_12, var_653);
        var_654 = wp::dot(var_652, var_16);
        // dist = -support                                                                    <L 270>
        var_655 = wp::neg(var_654);
        // pos = pos - 0.5 * dist * plane_normal                                              <L 271>
        var_657 = wp::mul(var_656, var_655);
        var_658 = wp::mul(var_657, var_plane_normal);
        var_659 = wp::sub(var_644, var_658);
        // contact_dist[contact_count] = dist                                                 <L 273>
        wp::assign_inplace(var_2, var_614, var_655);
        // contact_pos[contact_count] = pos                                                   <L 274>
        wp::assign_inplace(var_3, var_614, var_659);
        // contact_count = contact_count + 1                                                  <L 275>
        var_661 = wp::add(var_614, var_660);
    }
    var_662 = wp::where(var_631, var_661, var_614);
    var_663 = wp::where(var_631, var_654, var_615);
    var_664 = wp::where(var_631, var_655, var_616);
    var_665 = wp::where(var_631, var_659, var_611);
    // idx = indices[i]                                                                       <L 259>
    var_667 = wp::extract(var_21, var_666);
    // count = int(0)                                                                         <L 260>
    var_669 = wp::int(var_668);
    // for j in range(i + 1):                                                                 <L 261>
    var_671 = wp::add(var_666, var_670);
    var_672 = wp::range(var_671);
    start_for_33:;
        if (iter_cmp(var_672) == 0) goto end_for_33;
        var_673 = wp::iter_next(var_672);
        // if indices[j] == idx:                                                              <L 262>
        var_674 = wp::extract(var_21, var_673);
        var_675 = (var_674 == var_667);
        if (var_675) {
            // count = count + 1                                                              <L 263>
            var_677 = wp::add(var_669, var_676);
        }
        var_678 = wp::where(var_675, var_677, var_669);
        wp::assign(var_669, var_678);
        goto start_for_33;
    end_for_33:;
    // if count == 1:                                                                         <L 266>
    var_680 = (var_669 == var_679);
    if (var_680) {
        // pos = convex.vert[convex.vertadr + idx]                                            <L 267>
        var_681 = &((var_convex).vert);
        var_682 = &((var_convex).vertadr);
        var_684 = wp::load(var_682);
        var_683 = wp::add(var_684, var_667);
        var_686 = wp::load(var_681);
        var_685 = wp::address(var_686, var_683);
        var_688 = wp::load(var_685);
        var_687 = wp::copy(var_688);
        // pos = convex.pos + convex.rot @ pos                                                <L 268>
        var_689 = &((var_convex).pos);
        var_690 = &((var_convex).rot);
        var_692 = wp::load(var_690);
        var_691 = wp::mul(var_692, var_687);
        var_694 = wp::load(var_689);
        var_693 = wp::add(var_694, var_691);
        // support = wp.dot(plane_pos_local - convex.vert[convex.vertadr + idx], n)           <L 269>
        var_695 = &((var_convex).vert);
        var_696 = &((var_convex).vertadr);
        var_698 = wp::load(var_696);
        var_697 = wp::add(var_698, var_667);
        var_700 = wp::load(var_695);
        var_699 = wp::address(var_700, var_697);
        var_702 = wp::load(var_699);
        var_701 = wp::sub(var_12, var_702);
        var_703 = wp::dot(var_701, var_16);
        // dist = -support                                                                    <L 270>
        var_704 = wp::neg(var_703);
        // pos = pos - 0.5 * dist * plane_normal                                              <L 271>
        var_706 = wp::mul(var_705, var_704);
        var_707 = wp::mul(var_706, var_plane_normal);
        var_708 = wp::sub(var_693, var_707);
        // contact_dist[contact_count] = dist                                                 <L 273>
        wp::assign_inplace(var_2, var_662, var_704);
        // contact_pos[contact_count] = pos                                                   <L 274>
        wp::assign_inplace(var_3, var_662, var_708);
        // contact_count = contact_count + 1                                                  <L 275>
        var_710 = wp::add(var_662, var_709);
    }
    var_711 = wp::where(var_680, var_710, var_662);
    var_712 = wp::where(var_680, var_703, var_663);
    var_713 = wp::where(var_680, var_704, var_664);
    var_714 = wp::where(var_680, var_708, var_665);
    // idx = indices[i]                                                                       <L 259>
    var_716 = wp::extract(var_21, var_715);
    // count = int(0)                                                                         <L 260>
    var_718 = wp::int(var_717);
    // for j in range(i + 1):                                                                 <L 261>
    var_720 = wp::add(var_715, var_719);
    var_721 = wp::range(var_720);
    start_for_35:;
        if (iter_cmp(var_721) == 0) goto end_for_35;
        var_722 = wp::iter_next(var_721);
        // if indices[j] == idx:                                                              <L 262>
        var_723 = wp::extract(var_21, var_722);
        var_724 = (var_723 == var_716);
        if (var_724) {
            // count = count + 1                                                              <L 263>
            var_726 = wp::add(var_718, var_725);
        }
        var_727 = wp::where(var_724, var_726, var_718);
        wp::assign(var_718, var_727);
        goto start_for_35;
    end_for_35:;
    // if count == 1:                                                                         <L 266>
    var_729 = (var_718 == var_728);
    if (var_729) {
        // pos = convex.vert[convex.vertadr + idx]                                            <L 267>
        var_730 = &((var_convex).vert);
        var_731 = &((var_convex).vertadr);
        var_733 = wp::load(var_731);
        var_732 = wp::add(var_733, var_716);
        var_735 = wp::load(var_730);
        var_734 = wp::address(var_735, var_732);
        var_737 = wp::load(var_734);
        var_736 = wp::copy(var_737);
        // pos = convex.pos + convex.rot @ pos                                                <L 268>
        var_738 = &((var_convex).pos);
        var_739 = &((var_convex).rot);
        var_741 = wp::load(var_739);
        var_740 = wp::mul(var_741, var_736);
        var_743 = wp::load(var_738);
        var_742 = wp::add(var_743, var_740);
        // support = wp.dot(plane_pos_local - convex.vert[convex.vertadr + idx], n)           <L 269>
        var_744 = &((var_convex).vert);
        var_745 = &((var_convex).vertadr);
        var_747 = wp::load(var_745);
        var_746 = wp::add(var_747, var_716);
        var_749 = wp::load(var_744);
        var_748 = wp::address(var_749, var_746);
        var_751 = wp::load(var_748);
        var_750 = wp::sub(var_12, var_751);
        var_752 = wp::dot(var_750, var_16);
        // dist = -support                                                                    <L 270>
        var_753 = wp::neg(var_752);
        // pos = pos - 0.5 * dist * plane_normal                                              <L 271>
        var_755 = wp::mul(var_754, var_753);
        var_756 = wp::mul(var_755, var_plane_normal);
        var_757 = wp::sub(var_742, var_756);
        // contact_dist[contact_count] = dist                                                 <L 273>
        wp::assign_inplace(var_2, var_711, var_753);
        // contact_pos[contact_count] = pos                                                   <L 274>
        wp::assign_inplace(var_3, var_711, var_757);
        // contact_count = contact_count + 1                                                  <L 275>
        var_759 = wp::add(var_711, var_758);
    }
    var_760 = wp::where(var_729, var_759, var_711);
    var_761 = wp::where(var_729, var_752, var_712);
    var_762 = wp::where(var_729, var_753, var_713);
    var_763 = wp::where(var_729, var_757, var_714);
    // return contact_dist, contact_pos, plane_normal                                         <L 277>
    ret_0 = var_2;
    ret_1 = var_3;
    ret_2 = var_plane_normal;
    return;
}


// /home/cofallen/miniconda3/envs/mjlab/lib/python3.12/site-packages/mujoco_warp/_src/collision_primitive.py:809
static CUDA_CALLABLE void plane_convex_wrapper_0(
    wp::int32 var_naconmax_in,
    Geom_3242f8a8 var_plane,
    Geom_3242f8a8 var_convex,
    wp::int32 var_worldid,
    wp::float32 var_margin,
    wp::float32 var_gap,
    wp::int32 var_condim,
    wp::vec_t<5, wp::float32> var_friction,
    wp::vec_t<2, wp::float32> var_solref,
    wp::vec_t<2, wp::float32> var_solreffriction,
    wp::vec_t<5, wp::float32> var_solimp,
    wp::vec_t<2, wp::int32> var_geoms,
    wp::vec_t<2, wp::int32> var_pairid,
    wp::array_t<wp::float32> var_contact_dist_out,
    wp::array_t<wp::vec_t<3, wp::float32>> var_contact_pos_out,
    wp::array_t<wp::mat_t<3, 3, wp::float32>> var_contact_frame_out,
    wp::array_t<wp::float32> var_contact_includemargin_out,
    wp::array_t<wp::vec_t<5, wp::float32>> var_contact_friction_out,
    wp::array_t<wp::vec_t<2, wp::float32>> var_contact_solref_out,
    wp::array_t<wp::vec_t<2, wp::float32>> var_contact_solreffriction_out,
    wp::array_t<wp::vec_t<5, wp::float32>> var_contact_solimp_out,
    wp::array_t<wp::int32> var_contact_dim_out,
    wp::array_t<wp::vec_t<2, wp::int32>> var_contact_geom_out,
    wp::array_t<wp::int32> var_contact_efc_address_out,
    wp::array_t<wp::int32> var_contact_worldid_out,
    wp::array_t<wp::int32> var_contact_type_out,
    wp::array_t<wp::int32> var_contact_geomcollisionid_out,
    wp::array_t<wp::int32> var_nacon_out)
{
    //---------
    // primal vars
    wp::vec_t<3, wp::float32>* var_0;
    wp::vec_t<3, wp::float32>* var_1;
    wp::vec_t<4, wp::float32> var_2;
    wp::mat_t<4, 3, wp::float32> var_3;
    wp::vec_t<3, wp::float32> var_4;
    wp::vec_t<3, wp::float32> var_5;
    wp::vec_t<3, wp::float32> var_6;
    wp::mat_t<3, 3, wp::float32> var_7;
    const wp::int32 var_8 = 0;
    wp::float32 var_9;
    wp::vec_t<3, wp::float32> var_10;
    wp::int32 var_11;
    const wp::int32 var_12 = 1;
    wp::float32 var_13;
    wp::vec_t<3, wp::float32> var_14;
    wp::int32 var_15;
    const wp::int32 var_16 = 2;
    wp::float32 var_17;
    wp::vec_t<3, wp::float32> var_18;
    wp::int32 var_19;
    const wp::int32 var_20 = 3;
    wp::float32 var_21;
    wp::vec_t<3, wp::float32> var_22;
    wp::int32 var_23;
    //---------
    // forward
    // def plane_convex_wrapper(                                                              <L 810>
    // dist, pos, normal = plane_convex(plane.normal, plane.pos, convex)                      <L 844>
    var_0 = &((var_plane).normal);
    var_1 = &((var_plane).pos);
    var_5 = wp::load(var_0);
    var_6 = wp::load(var_1);
    plane_convex_0(var_5, var_6, var_convex, var_2, var_3, var_4);
    // frame = make_frame(normal)                                                             <L 846>
    var_7 = make_frame_0(var_4);
    // for i in range(4):                                                                     <L 847>
    // write_contact(                                                                         <L 848>
    // naconmax_in,                                                                           <L 849>
    // i,                                                                                     <L 850>
    // dist[i],                                                                               <L 851>
    var_9 = wp::extract(var_2, var_8);
    // pos[i],                                                                                <L 852>
    var_10 = wp::extract(var_3, var_8);
    // frame,                                                                                 <L 853>
    // margin,                                                                                <L 854>
    // gap,                                                                                   <L 855>
    // condim,                                                                                <L 856>
    // friction,                                                                              <L 857>
    // solref,                                                                                <L 858>
    // solreffriction,                                                                        <L 859>
    // solimp,                                                                                <L 860>
    // geoms,                                                                                 <L 861>
    // pairid,                                                                                <L 862>
    // worldid,                                                                               <L 863>
    // contact_dist_out,                                                                      <L 864>
    // contact_pos_out,                                                                       <L 865>
    // contact_frame_out,                                                                     <L 866>
    // contact_includemargin_out,                                                             <L 867>
    // contact_friction_out,                                                                  <L 868>
    // contact_solref_out,                                                                    <L 869>
    // contact_solreffriction_out,                                                            <L 870>
    // contact_solimp_out,                                                                    <L 871>
    // contact_dim_out,                                                                       <L 872>
    // contact_geom_out,                                                                      <L 873>
    // contact_efc_address_out,                                                               <L 874>
    // contact_worldid_out,                                                                   <L 875>
    // contact_type_out,                                                                      <L 876>
    // contact_geomcollisionid_out,                                                           <L 877>
    // nacon_out,                                                                             <L 878>
    var_11 = write_contact_0(var_naconmax_in, var_8, var_9, var_10, var_7, var_margin, var_gap, var_condim, var_friction, var_solref, var_solreffriction, var_solimp, var_geoms, var_pairid, var_worldid, var_contact_dist_out, var_contact_pos_out, var_contact_frame_out, var_contact_includemargin_out, var_contact_friction_out, var_contact_solref_out, var_contact_solreffriction_out, var_contact_solimp_out, var_contact_dim_out, var_contact_geom_out, var_contact_efc_address_out, var_contact_worldid_out, var_contact_type_out, var_contact_geomcollisionid_out, var_nacon_out);
    // write_contact(                                                                         <L 848>
    // naconmax_in,                                                                           <L 849>
    // i,                                                                                     <L 850>
    // dist[i],                                                                               <L 851>
    var_13 = wp::extract(var_2, var_12);
    // pos[i],                                                                                <L 852>
    var_14 = wp::extract(var_3, var_12);
    // frame,                                                                                 <L 853>
    // margin,                                                                                <L 854>
    // gap,                                                                                   <L 855>
    // condim,                                                                                <L 856>
    // friction,                                                                              <L 857>
    // solref,                                                                                <L 858>
    // solreffriction,                                                                        <L 859>
    // solimp,                                                                                <L 860>
    // geoms,                                                                                 <L 861>
    // pairid,                                                                                <L 862>
    // worldid,                                                                               <L 863>
    // contact_dist_out,                                                                      <L 864>
    // contact_pos_out,                                                                       <L 865>
    // contact_frame_out,                                                                     <L 866>
    // contact_includemargin_out,                                                             <L 867>
    // contact_friction_out,                                                                  <L 868>
    // contact_solref_out,                                                                    <L 869>
    // contact_solreffriction_out,                                                            <L 870>
    // contact_solimp_out,                                                                    <L 871>
    // contact_dim_out,                                                                       <L 872>
    // contact_geom_out,                                                                      <L 873>
    // contact_efc_address_out,                                                               <L 874>
    // contact_worldid_out,                                                                   <L 875>
    // contact_type_out,                                                                      <L 876>
    // contact_geomcollisionid_out,                                                           <L 877>
    // nacon_out,                                                                             <L 878>
    var_15 = write_contact_0(var_naconmax_in, var_12, var_13, var_14, var_7, var_margin, var_gap, var_condim, var_friction, var_solref, var_solreffriction, var_solimp, var_geoms, var_pairid, var_worldid, var_contact_dist_out, var_contact_pos_out, var_contact_frame_out, var_contact_includemargin_out, var_contact_friction_out, var_contact_solref_out, var_contact_solreffriction_out, var_contact_solimp_out, var_contact_dim_out, var_contact_geom_out, var_contact_efc_address_out, var_contact_worldid_out, var_contact_type_out, var_contact_geomcollisionid_out, var_nacon_out);
    // write_contact(                                                                         <L 848>
    // naconmax_in,                                                                           <L 849>
    // i,                                                                                     <L 850>
    // dist[i],                                                                               <L 851>
    var_17 = wp::extract(var_2, var_16);
    // pos[i],                                                                                <L 852>
    var_18 = wp::extract(var_3, var_16);
    // frame,                                                                                 <L 853>
    // margin,                                                                                <L 854>
    // gap,                                                                                   <L 855>
    // condim,                                                                                <L 856>
    // friction,                                                                              <L 857>
    // solref,                                                                                <L 858>
    // solreffriction,                                                                        <L 859>
    // solimp,                                                                                <L 860>
    // geoms,                                                                                 <L 861>
    // pairid,                                                                                <L 862>
    // worldid,                                                                               <L 863>
    // contact_dist_out,                                                                      <L 864>
    // contact_pos_out,                                                                       <L 865>
    // contact_frame_out,                                                                     <L 866>
    // contact_includemargin_out,                                                             <L 867>
    // contact_friction_out,                                                                  <L 868>
    // contact_solref_out,                                                                    <L 869>
    // contact_solreffriction_out,                                                            <L 870>
    // contact_solimp_out,                                                                    <L 871>
    // contact_dim_out,                                                                       <L 872>
    // contact_geom_out,                                                                      <L 873>
    // contact_efc_address_out,                                                               <L 874>
    // contact_worldid_out,                                                                   <L 875>
    // contact_type_out,                                                                      <L 876>
    // contact_geomcollisionid_out,                                                           <L 877>
    // nacon_out,                                                                             <L 878>
    var_19 = write_contact_0(var_naconmax_in, var_16, var_17, var_18, var_7, var_margin, var_gap, var_condim, var_friction, var_solref, var_solreffriction, var_solimp, var_geoms, var_pairid, var_worldid, var_contact_dist_out, var_contact_pos_out, var_contact_frame_out, var_contact_includemargin_out, var_contact_friction_out, var_contact_solref_out, var_contact_solreffriction_out, var_contact_solimp_out, var_contact_dim_out, var_contact_geom_out, var_contact_efc_address_out, var_contact_worldid_out, var_contact_type_out, var_contact_geomcollisionid_out, var_nacon_out);
    // write_contact(                                                                         <L 848>
    // naconmax_in,                                                                           <L 849>
    // i,                                                                                     <L 850>
    // dist[i],                                                                               <L 851>
    var_21 = wp::extract(var_2, var_20);
    // pos[i],                                                                                <L 852>
    var_22 = wp::extract(var_3, var_20);
    // frame,                                                                                 <L 853>
    // margin,                                                                                <L 854>
    // gap,                                                                                   <L 855>
    // condim,                                                                                <L 856>
    // friction,                                                                              <L 857>
    // solref,                                                                                <L 858>
    // solreffriction,                                                                        <L 859>
    // solimp,                                                                                <L 860>
    // geoms,                                                                                 <L 861>
    // pairid,                                                                                <L 862>
    // worldid,                                                                               <L 863>
    // contact_dist_out,                                                                      <L 864>
    // contact_pos_out,                                                                       <L 865>
    // contact_frame_out,                                                                     <L 866>
    // contact_includemargin_out,                                                             <L 867>
    // contact_friction_out,                                                                  <L 868>
    // contact_solref_out,                                                                    <L 869>
    // contact_solreffriction_out,                                                            <L 870>
    // contact_solimp_out,                                                                    <L 871>
    // contact_dim_out,                                                                       <L 872>
    // contact_geom_out,                                                                      <L 873>
    // contact_efc_address_out,                                                               <L 874>
    // contact_worldid_out,                                                                   <L 875>
    // contact_type_out,                                                                      <L 876>
    // contact_geomcollisionid_out,                                                           <L 877>
    // nacon_out,                                                                             <L 878>
    var_23 = write_contact_0(var_naconmax_in, var_20, var_21, var_22, var_7, var_margin, var_gap, var_condim, var_friction, var_solref, var_solreffriction, var_solimp, var_geoms, var_pairid, var_worldid, var_contact_dist_out, var_contact_pos_out, var_contact_frame_out, var_contact_includemargin_out, var_contact_friction_out, var_contact_solref_out, var_contact_solreffriction_out, var_contact_solimp_out, var_contact_dim_out, var_contact_geom_out, var_contact_efc_address_out, var_contact_worldid_out, var_contact_type_out, var_contact_geomcollisionid_out, var_nacon_out);
}


// /home/cofallen/miniconda3/envs/mjlab/lib/python3.12/site-packages/mujoco_warp/_src/collision_core.py:290
static CUDA_CALLABLE void adj_contact_margin_gap_0(
    wp::array_t<wp::float32> var_geom_margin,
    wp::array_t<wp::float32> var_geom_gap,
    wp::array_t<wp::float32> var_pair_margin,
    wp::array_t<wp::float32> var_pair_gap,
    wp::vec_t<2, wp::int32> var_geoms,
    wp::int32 var_pairid,
    wp::int32 var_worldid,
    wp::float32 & ret_0,
    wp::float32 & ret_1,
    wp::array_t<wp::float32> & adj_geom_margin,
    wp::array_t<wp::float32> & adj_geom_gap,
    wp::array_t<wp::float32> & adj_pair_margin,
    wp::array_t<wp::float32> & adj_pair_gap,
    wp::vec_t<2, wp::int32> & adj_geoms,
    wp::int32 & adj_pairid,
    wp::int32 & adj_worldid,
    wp::float32 & adj_ret_0,
    wp::float32 & adj_ret_1)
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


// /home/cofallen/miniconda3/envs/mjlab/lib/python3.12/site-packages/mujoco_warp/_src/collision_core.py:316
static CUDA_CALLABLE void adj_contact_material_params_0(
    wp::array_t<wp::int32> var_geom_condim,
    wp::array_t<wp::int32> var_geom_priority,
    wp::array_t<wp::float32> var_geom_solmix,
    wp::array_t<wp::vec_t<2, wp::float32>> var_geom_solref,
    wp::array_t<wp::vec_t<5, wp::float32>> var_geom_solimp,
    wp::array_t<wp::vec_t<3, wp::float32>> var_geom_friction,
    wp::array_t<wp::int32> var_pair_dim,
    wp::array_t<wp::vec_t<2, wp::float32>> var_pair_solref,
    wp::array_t<wp::vec_t<2, wp::float32>> var_pair_solreffriction,
    wp::array_t<wp::vec_t<5, wp::float32>> var_pair_solimp,
    wp::array_t<wp::vec_t<5, wp::float32>> var_pair_friction,
    wp::vec_t<2, wp::int32> var_geoms,
    wp::int32 var_pairid,
    wp::int32 var_worldid,
    wp::int32 & ret_0,
    wp::vec_t<5, wp::float32> & ret_1,
    wp::vec_t<2, wp::float32> & ret_2,
    wp::vec_t<2, wp::float32> & ret_3,
    wp::vec_t<5, wp::float32> & ret_4,
    wp::array_t<wp::int32> & adj_geom_condim,
    wp::array_t<wp::int32> & adj_geom_priority,
    wp::array_t<wp::float32> & adj_geom_solmix,
    wp::array_t<wp::vec_t<2, wp::float32>> & adj_geom_solref,
    wp::array_t<wp::vec_t<5, wp::float32>> & adj_geom_solimp,
    wp::array_t<wp::vec_t<3, wp::float32>> & adj_geom_friction,
    wp::array_t<wp::int32> & adj_pair_dim,
    wp::array_t<wp::vec_t<2, wp::float32>> & adj_pair_solref,
    wp::array_t<wp::vec_t<2, wp::float32>> & adj_pair_solreffriction,
    wp::array_t<wp::vec_t<5, wp::float32>> & adj_pair_solimp,
    wp::array_t<wp::vec_t<5, wp::float32>> & adj_pair_friction,
    wp::vec_t<2, wp::int32> & adj_geoms,
    wp::int32 & adj_pairid,
    wp::int32 & adj_worldid,
    wp::int32 & adj_ret_0,
    wp::vec_t<5, wp::float32> & adj_ret_1,
    wp::vec_t<2, wp::float32> & adj_ret_2,
    wp::vec_t<2, wp::float32> & adj_ret_3,
    wp::vec_t<5, wp::float32> & adj_ret_4)
{
	// reverse mode disabled (no backward-enabled kernel depends on this function)
}


// /home/cofallen/miniconda3/envs/mjlab/lib/python3.12/site-packages/mujoco_warp/_src/collision_core.py:402
static CUDA_CALLABLE void adj_contact_params_0(
    wp::array_t<wp::int32> var_geom_condim,
    wp::array_t<wp::int32> var_geom_priority,
    wp::array_t<wp::float32> var_geom_solmix,
    wp::array_t<wp::vec_t<2, wp::float32>> var_geom_solref,
    wp::array_t<wp::vec_t<5, wp::float32>> var_geom_solimp,
    wp::array_t<wp::vec_t<3, wp::float32>> var_geom_friction,
    wp::array_t<wp::float32> var_geom_margin,
    wp::array_t<wp::float32> var_geom_gap,
    wp::array_t<wp::int32> var_pair_dim,
    wp::array_t<wp::vec_t<2, wp::float32>> var_pair_solref,
    wp::array_t<wp::vec_t<2, wp::float32>> var_pair_solreffriction,
    wp::array_t<wp::vec_t<5, wp::float32>> var_pair_solimp,
    wp::array_t<wp::float32> var_pair_margin,
    wp::array_t<wp::float32> var_pair_gap,
    wp::array_t<wp::vec_t<5, wp::float32>> var_pair_friction,
    wp::array_t<wp::vec_t<2, wp::int32>> var_collision_pair_in,
    wp::array_t<wp::vec_t<2, wp::int32>> var_collision_pairid_in,
    wp::int32 var_cid,
    wp::int32 var_worldid,
    wp::vec_t<2, wp::int32> & ret_0,
    wp::float32 & ret_1,
    wp::float32 & ret_2,
    wp::int32 & ret_3,
    wp::vec_t<5, wp::float32> & ret_4,
    wp::vec_t<2, wp::float32> & ret_5,
    wp::vec_t<2, wp::float32> & ret_6,
    wp::vec_t<5, wp::float32> & ret_7,
    wp::array_t<wp::int32> & adj_geom_condim,
    wp::array_t<wp::int32> & adj_geom_priority,
    wp::array_t<wp::float32> & adj_geom_solmix,
    wp::array_t<wp::vec_t<2, wp::float32>> & adj_geom_solref,
    wp::array_t<wp::vec_t<5, wp::float32>> & adj_geom_solimp,
    wp::array_t<wp::vec_t<3, wp::float32>> & adj_geom_friction,
    wp::array_t<wp::float32> & adj_geom_margin,
    wp::array_t<wp::float32> & adj_geom_gap,
    wp::array_t<wp::int32> & adj_pair_dim,
    wp::array_t<wp::vec_t<2, wp::float32>> & adj_pair_solref,
    wp::array_t<wp::vec_t<2, wp::float32>> & adj_pair_solreffriction,
    wp::array_t<wp::vec_t<5, wp::float32>> & adj_pair_solimp,
    wp::array_t<wp::float32> & adj_pair_margin,
    wp::array_t<wp::float32> & adj_pair_gap,
    wp::array_t<wp::vec_t<5, wp::float32>> & adj_pair_friction,
    wp::array_t<wp::vec_t<2, wp::int32>> & adj_collision_pair_in,
    wp::array_t<wp::vec_t<2, wp::int32>> & adj_collision_pairid_in,
    wp::int32 & adj_cid,
    wp::int32 & adj_worldid,
    wp::vec_t<2, wp::int32> & adj_ret_0,
    wp::float32 & adj_ret_1,
    wp::float32 & adj_ret_2,
    wp::int32 & adj_ret_3,
    wp::vec_t<5, wp::float32> & adj_ret_4,
    wp::vec_t<2, wp::float32> & adj_ret_5,
    wp::vec_t<2, wp::float32> & adj_ret_6,
    wp::vec_t<5, wp::float32> & adj_ret_7)
{
	// reverse mode disabled (no backward-enabled kernel depends on this function)
}


// /home/cofallen/miniconda3/envs/mjlab/lib/python3.12/site-packages/mujoco_warp/_src/collision_core.py:65
static CUDA_CALLABLE void adj_geom_collision_pair_from_types_0(
    wp::array_t<wp::int32> var_geom_dataid,
    wp::array_t<wp::vec_t<3, wp::float32>> var_geom_size,
    wp::array_t<wp::int32> var_mesh_vertadr,
    wp::array_t<wp::int32> var_mesh_vertnum,
    wp::array_t<wp::int32> var_mesh_graphadr,
    wp::array_t<wp::vec_t<3, wp::float32>> var_mesh_vert,
    wp::array_t<wp::int32> var_mesh_graph,
    wp::array_t<wp::int32> var_mesh_polynum,
    wp::array_t<wp::int32> var_mesh_polyadr,
    wp::array_t<wp::vec_t<3, wp::float32>> var_mesh_polynormal,
    wp::array_t<wp::int32> var_mesh_polyvertadr,
    wp::array_t<wp::int32> var_mesh_polyvertnum,
    wp::array_t<wp::int32> var_mesh_polyvert,
    wp::array_t<wp::int32> var_mesh_polymapadr,
    wp::array_t<wp::int32> var_mesh_polymapnum,
    wp::array_t<wp::int32> var_mesh_polymap,
    wp::array_t<wp::vec_t<3, wp::float32>> var_geom_xpos_in,
    wp::array_t<wp::mat_t<3, 3, wp::float32>> var_geom_xmat_in,
    wp::int32 var_geom_type1,
    wp::int32 var_geom_type2,
    wp::vec_t<2, wp::int32> var_geoms,
    wp::int32 var_worldid,
    Geom_3242f8a8 & ret_0,
    Geom_3242f8a8 & ret_1,
    wp::array_t<wp::int32> & adj_geom_dataid,
    wp::array_t<wp::vec_t<3, wp::float32>> & adj_geom_size,
    wp::array_t<wp::int32> & adj_mesh_vertadr,
    wp::array_t<wp::int32> & adj_mesh_vertnum,
    wp::array_t<wp::int32> & adj_mesh_graphadr,
    wp::array_t<wp::vec_t<3, wp::float32>> & adj_mesh_vert,
    wp::array_t<wp::int32> & adj_mesh_graph,
    wp::array_t<wp::int32> & adj_mesh_polynum,
    wp::array_t<wp::int32> & adj_mesh_polyadr,
    wp::array_t<wp::vec_t<3, wp::float32>> & adj_mesh_polynormal,
    wp::array_t<wp::int32> & adj_mesh_polyvertadr,
    wp::array_t<wp::int32> & adj_mesh_polyvertnum,
    wp::array_t<wp::int32> & adj_mesh_polyvert,
    wp::array_t<wp::int32> & adj_mesh_polymapadr,
    wp::array_t<wp::int32> & adj_mesh_polymapnum,
    wp::array_t<wp::int32> & adj_mesh_polymap,
    wp::array_t<wp::vec_t<3, wp::float32>> & adj_geom_xpos_in,
    wp::array_t<wp::mat_t<3, 3, wp::float32>> & adj_geom_xmat_in,
    wp::int32 & adj_geom_type1,
    wp::int32 & adj_geom_type2,
    wp::vec_t<2, wp::int32> & adj_geoms,
    wp::int32 & adj_worldid,
    Geom_3242f8a8 & adj_ret_0,
    Geom_3242f8a8 & adj_ret_1)
{
	// reverse mode disabled (no backward-enabled kernel depends on this function)
}


// /home/cofallen/miniconda3/envs/mjlab/lib/python3.12/site-packages/mujoco_warp/_src/collision_core.py:158
static CUDA_CALLABLE void adj_geom_collision_pair_0(
    wp::array_t<wp::int32> var_geom_type,
    wp::array_t<wp::int32> var_geom_dataid,
    wp::array_t<wp::vec_t<3, wp::float32>> var_geom_size,
    wp::array_t<wp::int32> var_mesh_vertadr,
    wp::array_t<wp::int32> var_mesh_vertnum,
    wp::array_t<wp::int32> var_mesh_graphadr,
    wp::array_t<wp::vec_t<3, wp::float32>> var_mesh_vert,
    wp::array_t<wp::int32> var_mesh_graph,
    wp::array_t<wp::int32> var_mesh_polynum,
    wp::array_t<wp::int32> var_mesh_polyadr,
    wp::array_t<wp::vec_t<3, wp::float32>> var_mesh_polynormal,
    wp::array_t<wp::int32> var_mesh_polyvertadr,
    wp::array_t<wp::int32> var_mesh_polyvertnum,
    wp::array_t<wp::int32> var_mesh_polyvert,
    wp::array_t<wp::int32> var_mesh_polymapadr,
    wp::array_t<wp::int32> var_mesh_polymapnum,
    wp::array_t<wp::int32> var_mesh_polymap,
    wp::array_t<wp::vec_t<3, wp::float32>> var_geom_xpos_in,
    wp::array_t<wp::mat_t<3, 3, wp::float32>> var_geom_xmat_in,
    wp::vec_t<2, wp::int32> var_geoms,
    wp::int32 var_worldid,
    Geom_3242f8a8 & ret_0,
    Geom_3242f8a8 & ret_1,
    wp::array_t<wp::int32> & adj_geom_type,
    wp::array_t<wp::int32> & adj_geom_dataid,
    wp::array_t<wp::vec_t<3, wp::float32>> & adj_geom_size,
    wp::array_t<wp::int32> & adj_mesh_vertadr,
    wp::array_t<wp::int32> & adj_mesh_vertnum,
    wp::array_t<wp::int32> & adj_mesh_graphadr,
    wp::array_t<wp::vec_t<3, wp::float32>> & adj_mesh_vert,
    wp::array_t<wp::int32> & adj_mesh_graph,
    wp::array_t<wp::int32> & adj_mesh_polynum,
    wp::array_t<wp::int32> & adj_mesh_polyadr,
    wp::array_t<wp::vec_t<3, wp::float32>> & adj_mesh_polynormal,
    wp::array_t<wp::int32> & adj_mesh_polyvertadr,
    wp::array_t<wp::int32> & adj_mesh_polyvertnum,
    wp::array_t<wp::int32> & adj_mesh_polyvert,
    wp::array_t<wp::int32> & adj_mesh_polymapadr,
    wp::array_t<wp::int32> & adj_mesh_polymapnum,
    wp::array_t<wp::int32> & adj_mesh_polymap,
    wp::array_t<wp::vec_t<3, wp::float32>> & adj_geom_xpos_in,
    wp::array_t<wp::mat_t<3, 3, wp::float32>> & adj_geom_xmat_in,
    wp::vec_t<2, wp::int32> & adj_geoms,
    wp::int32 & adj_worldid,
    Geom_3242f8a8 & adj_ret_0,
    Geom_3242f8a8 & adj_ret_1)
{
	// reverse mode disabled (no backward-enabled kernel depends on this function)
}


// /home/cofallen/miniconda3/envs/mjlab/lib/python3.12/site-packages/mujoco_warp/_src/collision_primitive_core.py:459
static CUDA_CALLABLE void adj_plane_cylinder_0(
    wp::vec_t<3, wp::float32> var_plane_normal,
    wp::vec_t<3, wp::float32> var_plane_pos,
    wp::vec_t<3, wp::float32> var_cylinder_center,
    wp::vec_t<3, wp::float32> var_cylinder_axis,
    wp::float32 var_cylinder_radius,
    wp::float32 var_cylinder_half_height,
    wp::vec_t<4, wp::float32> & ret_0,
    wp::mat_t<4, 3, wp::float32> & ret_1,
    wp::vec_t<3, wp::float32> & ret_2,
    wp::vec_t<3, wp::float32> & adj_plane_normal,
    wp::vec_t<3, wp::float32> & adj_plane_pos,
    wp::vec_t<3, wp::float32> & adj_cylinder_center,
    wp::vec_t<3, wp::float32> & adj_cylinder_axis,
    wp::float32 & adj_cylinder_radius,
    wp::float32 & adj_cylinder_half_height,
    wp::vec_t<4, wp::float32> & adj_ret_0,
    wp::mat_t<4, 3, wp::float32> & adj_ret_1,
    wp::vec_t<3, wp::float32> & adj_ret_2)
{
	// reverse mode disabled (no backward-enabled kernel depends on this function)
}


// /home/cofallen/miniconda3/envs/mjlab/lib/python3.12/site-packages/mujoco_warp/_src/math.py:202
static CUDA_CALLABLE void adj_orthogonals_0(
    wp::vec_t<3, wp::float32> var_a,
    wp::vec_t<3, wp::float32> & ret_0,
    wp::vec_t<3, wp::float32> & ret_1,
    wp::vec_t<3, wp::float32> & adj_a,
    wp::vec_t<3, wp::float32> & adj_ret_0,
    wp::vec_t<3, wp::float32> & adj_ret_1)
{
	// reverse mode disabled (no backward-enabled kernel depends on this function)
}


// /home/cofallen/miniconda3/envs/mjlab/lib/python3.12/site-packages/mujoco_warp/_src/math.py:246
static CUDA_CALLABLE void adj_make_frame_0(
    wp::vec_t<3, wp::float32> var_a,
    wp::vec_t<3, wp::float32> & adj_a,
    wp::mat_t<3, 3, wp::float32> & adj_ret)
{
	// reverse mode disabled (no backward-enabled kernel depends on this function)
}


// /home/cofallen/miniconda3/envs/mjlab/lib/python3.12/site-packages/mujoco_warp/_src/collision_core.py:213
static CUDA_CALLABLE void adj_write_contact_0(
    wp::int32 var_naconmax_in,
    wp::int32 var_id_,
    wp::float32 var_dist_in,
    wp::vec_t<3, wp::float32> var_pos_in,
    wp::mat_t<3, 3, wp::float32> var_frame_in,
    wp::float32 var_margin_in,
    wp::float32 var_gap_in,
    wp::int32 var_condim_in,
    wp::vec_t<5, wp::float32> var_friction_in,
    wp::vec_t<2, wp::float32> var_solref_in,
    wp::vec_t<2, wp::float32> var_solreffriction_in,
    wp::vec_t<5, wp::float32> var_solimp_in,
    wp::vec_t<2, wp::int32> var_geoms_in,
    wp::vec_t<2, wp::int32> var_pairid_in,
    wp::int32 var_worldid_in,
    wp::array_t<wp::float32> var_contact_dist_out,
    wp::array_t<wp::vec_t<3, wp::float32>> var_contact_pos_out,
    wp::array_t<wp::mat_t<3, 3, wp::float32>> var_contact_frame_out,
    wp::array_t<wp::float32> var_contact_includemargin_out,
    wp::array_t<wp::vec_t<5, wp::float32>> var_contact_friction_out,
    wp::array_t<wp::vec_t<2, wp::float32>> var_contact_solref_out,
    wp::array_t<wp::vec_t<2, wp::float32>> var_contact_solreffriction_out,
    wp::array_t<wp::vec_t<5, wp::float32>> var_contact_solimp_out,
    wp::array_t<wp::int32> var_contact_dim_out,
    wp::array_t<wp::vec_t<2, wp::int32>> var_contact_geom_out,
    wp::array_t<wp::int32> var_contact_efc_address_out,
    wp::array_t<wp::int32> var_contact_worldid_out,
    wp::array_t<wp::int32> var_contact_type_out,
    wp::array_t<wp::int32> var_contact_geomcollisionid_out,
    wp::array_t<wp::int32> var_nacon_out,
    wp::int32 & adj_naconmax_in,
    wp::int32 & adj_id_,
    wp::float32 & adj_dist_in,
    wp::vec_t<3, wp::float32> & adj_pos_in,
    wp::mat_t<3, 3, wp::float32> & adj_frame_in,
    wp::float32 & adj_margin_in,
    wp::float32 & adj_gap_in,
    wp::int32 & adj_condim_in,
    wp::vec_t<5, wp::float32> & adj_friction_in,
    wp::vec_t<2, wp::float32> & adj_solref_in,
    wp::vec_t<2, wp::float32> & adj_solreffriction_in,
    wp::vec_t<5, wp::float32> & adj_solimp_in,
    wp::vec_t<2, wp::int32> & adj_geoms_in,
    wp::vec_t<2, wp::int32> & adj_pairid_in,
    wp::int32 & adj_worldid_in,
    wp::array_t<wp::float32> & adj_contact_dist_out,
    wp::array_t<wp::vec_t<3, wp::float32>> & adj_contact_pos_out,
    wp::array_t<wp::mat_t<3, 3, wp::float32>> & adj_contact_frame_out,
    wp::array_t<wp::float32> & adj_contact_includemargin_out,
    wp::array_t<wp::vec_t<5, wp::float32>> & adj_contact_friction_out,
    wp::array_t<wp::vec_t<2, wp::float32>> & adj_contact_solref_out,
    wp::array_t<wp::vec_t<2, wp::float32>> & adj_contact_solreffriction_out,
    wp::array_t<wp::vec_t<5, wp::float32>> & adj_contact_solimp_out,
    wp::array_t<wp::int32> & adj_contact_dim_out,
    wp::array_t<wp::vec_t<2, wp::int32>> & adj_contact_geom_out,
    wp::array_t<wp::int32> & adj_contact_efc_address_out,
    wp::array_t<wp::int32> & adj_contact_worldid_out,
    wp::array_t<wp::int32> & adj_contact_type_out,
    wp::array_t<wp::int32> & adj_contact_geomcollisionid_out,
    wp::array_t<wp::int32> & adj_nacon_out,
    wp::int32 & adj_ret)
{
	// reverse mode disabled (no backward-enabled kernel depends on this function)
}


// /home/cofallen/miniconda3/envs/mjlab/lib/python3.12/site-packages/mujoco_warp/_src/collision_primitive.py:963
static CUDA_CALLABLE void adj_plane_cylinder_wrapper_0(
    wp::int32 var_naconmax_in,
    Geom_3242f8a8 var_plane,
    Geom_3242f8a8 var_cylinder,
    wp::int32 var_worldid,
    wp::float32 var_margin,
    wp::float32 var_gap,
    wp::int32 var_condim,
    wp::vec_t<5, wp::float32> var_friction,
    wp::vec_t<2, wp::float32> var_solref,
    wp::vec_t<2, wp::float32> var_solreffriction,
    wp::vec_t<5, wp::float32> var_solimp,
    wp::vec_t<2, wp::int32> var_geoms,
    wp::vec_t<2, wp::int32> var_pairid,
    wp::array_t<wp::float32> var_contact_dist_out,
    wp::array_t<wp::vec_t<3, wp::float32>> var_contact_pos_out,
    wp::array_t<wp::mat_t<3, 3, wp::float32>> var_contact_frame_out,
    wp::array_t<wp::float32> var_contact_includemargin_out,
    wp::array_t<wp::vec_t<5, wp::float32>> var_contact_friction_out,
    wp::array_t<wp::vec_t<2, wp::float32>> var_contact_solref_out,
    wp::array_t<wp::vec_t<2, wp::float32>> var_contact_solreffriction_out,
    wp::array_t<wp::vec_t<5, wp::float32>> var_contact_solimp_out,
    wp::array_t<wp::int32> var_contact_dim_out,
    wp::array_t<wp::vec_t<2, wp::int32>> var_contact_geom_out,
    wp::array_t<wp::int32> var_contact_efc_address_out,
    wp::array_t<wp::int32> var_contact_worldid_out,
    wp::array_t<wp::int32> var_contact_type_out,
    wp::array_t<wp::int32> var_contact_geomcollisionid_out,
    wp::array_t<wp::int32> var_nacon_out,
    wp::int32 & adj_naconmax_in,
    Geom_3242f8a8 & adj_plane,
    Geom_3242f8a8 & adj_cylinder,
    wp::int32 & adj_worldid,
    wp::float32 & adj_margin,
    wp::float32 & adj_gap,
    wp::int32 & adj_condim,
    wp::vec_t<5, wp::float32> & adj_friction,
    wp::vec_t<2, wp::float32> & adj_solref,
    wp::vec_t<2, wp::float32> & adj_solreffriction,
    wp::vec_t<5, wp::float32> & adj_solimp,
    wp::vec_t<2, wp::int32> & adj_geoms,
    wp::vec_t<2, wp::int32> & adj_pairid,
    wp::array_t<wp::float32> & adj_contact_dist_out,
    wp::array_t<wp::vec_t<3, wp::float32>> & adj_contact_pos_out,
    wp::array_t<wp::mat_t<3, 3, wp::float32>> & adj_contact_frame_out,
    wp::array_t<wp::float32> & adj_contact_includemargin_out,
    wp::array_t<wp::vec_t<5, wp::float32>> & adj_contact_friction_out,
    wp::array_t<wp::vec_t<2, wp::float32>> & adj_contact_solref_out,
    wp::array_t<wp::vec_t<2, wp::float32>> & adj_contact_solreffriction_out,
    wp::array_t<wp::vec_t<5, wp::float32>> & adj_contact_solimp_out,
    wp::array_t<wp::int32> & adj_contact_dim_out,
    wp::array_t<wp::vec_t<2, wp::int32>> & adj_contact_geom_out,
    wp::array_t<wp::int32> & adj_contact_efc_address_out,
    wp::array_t<wp::int32> & adj_contact_worldid_out,
    wp::array_t<wp::int32> & adj_contact_type_out,
    wp::array_t<wp::int32> & adj_contact_geomcollisionid_out,
    wp::array_t<wp::int32> & adj_nacon_out)
{
	// reverse mode disabled (no backward-enabled kernel depends on this function)
}


// /home/cofallen/miniconda3/envs/mjlab/lib/python3.12/site-packages/mujoco_warp/_src/collision_primitive.py:51
static CUDA_CALLABLE void adj_plane_convex_0(
    wp::vec_t<3, wp::float32> var_plane_normal,
    wp::vec_t<3, wp::float32> var_plane_pos,
    Geom_3242f8a8 var_convex,
    wp::vec_t<4, wp::float32> & ret_0,
    wp::mat_t<4, 3, wp::float32> & ret_1,
    wp::vec_t<3, wp::float32> & ret_2,
    wp::vec_t<3, wp::float32> & adj_plane_normal,
    wp::vec_t<3, wp::float32> & adj_plane_pos,
    Geom_3242f8a8 & adj_convex,
    wp::vec_t<4, wp::float32> & adj_ret_0,
    wp::mat_t<4, 3, wp::float32> & adj_ret_1,
    wp::vec_t<3, wp::float32> & adj_ret_2)
{
	// reverse mode disabled (no backward-enabled kernel depends on this function)
}


// /home/cofallen/miniconda3/envs/mjlab/lib/python3.12/site-packages/mujoco_warp/_src/collision_primitive.py:809
static CUDA_CALLABLE void adj_plane_convex_wrapper_0(
    wp::int32 var_naconmax_in,
    Geom_3242f8a8 var_plane,
    Geom_3242f8a8 var_convex,
    wp::int32 var_worldid,
    wp::float32 var_margin,
    wp::float32 var_gap,
    wp::int32 var_condim,
    wp::vec_t<5, wp::float32> var_friction,
    wp::vec_t<2, wp::float32> var_solref,
    wp::vec_t<2, wp::float32> var_solreffriction,
    wp::vec_t<5, wp::float32> var_solimp,
    wp::vec_t<2, wp::int32> var_geoms,
    wp::vec_t<2, wp::int32> var_pairid,
    wp::array_t<wp::float32> var_contact_dist_out,
    wp::array_t<wp::vec_t<3, wp::float32>> var_contact_pos_out,
    wp::array_t<wp::mat_t<3, 3, wp::float32>> var_contact_frame_out,
    wp::array_t<wp::float32> var_contact_includemargin_out,
    wp::array_t<wp::vec_t<5, wp::float32>> var_contact_friction_out,
    wp::array_t<wp::vec_t<2, wp::float32>> var_contact_solref_out,
    wp::array_t<wp::vec_t<2, wp::float32>> var_contact_solreffriction_out,
    wp::array_t<wp::vec_t<5, wp::float32>> var_contact_solimp_out,
    wp::array_t<wp::int32> var_contact_dim_out,
    wp::array_t<wp::vec_t<2, wp::int32>> var_contact_geom_out,
    wp::array_t<wp::int32> var_contact_efc_address_out,
    wp::array_t<wp::int32> var_contact_worldid_out,
    wp::array_t<wp::int32> var_contact_type_out,
    wp::array_t<wp::int32> var_contact_geomcollisionid_out,
    wp::array_t<wp::int32> var_nacon_out,
    wp::int32 & adj_naconmax_in,
    Geom_3242f8a8 & adj_plane,
    Geom_3242f8a8 & adj_convex,
    wp::int32 & adj_worldid,
    wp::float32 & adj_margin,
    wp::float32 & adj_gap,
    wp::int32 & adj_condim,
    wp::vec_t<5, wp::float32> & adj_friction,
    wp::vec_t<2, wp::float32> & adj_solref,
    wp::vec_t<2, wp::float32> & adj_solreffriction,
    wp::vec_t<5, wp::float32> & adj_solimp,
    wp::vec_t<2, wp::int32> & adj_geoms,
    wp::vec_t<2, wp::int32> & adj_pairid,
    wp::array_t<wp::float32> & adj_contact_dist_out,
    wp::array_t<wp::vec_t<3, wp::float32>> & adj_contact_pos_out,
    wp::array_t<wp::mat_t<3, 3, wp::float32>> & adj_contact_frame_out,
    wp::array_t<wp::float32> & adj_contact_includemargin_out,
    wp::array_t<wp::vec_t<5, wp::float32>> & adj_contact_friction_out,
    wp::array_t<wp::vec_t<2, wp::float32>> & adj_contact_solref_out,
    wp::array_t<wp::vec_t<2, wp::float32>> & adj_contact_solreffriction_out,
    wp::array_t<wp::vec_t<5, wp::float32>> & adj_contact_solimp_out,
    wp::array_t<wp::int32> & adj_contact_dim_out,
    wp::array_t<wp::vec_t<2, wp::int32>> & adj_contact_geom_out,
    wp::array_t<wp::int32> & adj_contact_efc_address_out,
    wp::array_t<wp::int32> & adj_contact_worldid_out,
    wp::array_t<wp::int32> & adj_contact_type_out,
    wp::array_t<wp::int32> & adj_contact_geomcollisionid_out,
    wp::array_t<wp::int32> & adj_nacon_out)
{
	// reverse mode disabled (no backward-enabled kernel depends on this function)
}



extern "C" __global__ void _primitive_narrowphase__locals__primitive_narrowphase_5c2524eb_cuda_kernel_forward(
    wp::launch_bounds_t<1> dim,
    wp::array_t<wp::int32> var_geom_type,
    wp::array_t<wp::int32> var_geom_condim,
    wp::array_t<wp::int32> var_geom_dataid,
    wp::array_t<wp::int32> var_geom_priority,
    wp::array_t<wp::float32> var_geom_solmix,
    wp::array_t<wp::vec_t<2, wp::float32>> var_geom_solref,
    wp::array_t<wp::vec_t<5, wp::float32>> var_geom_solimp,
    wp::array_t<wp::vec_t<3, wp::float32>> var_geom_size,
    wp::array_t<wp::vec_t<3, wp::float32>> var_geom_friction,
    wp::array_t<wp::float32> var_geom_margin,
    wp::array_t<wp::float32> var_geom_gap,
    wp::array_t<wp::int32> var_mesh_vertadr,
    wp::array_t<wp::int32> var_mesh_vertnum,
    wp::array_t<wp::int32> var_mesh_graphadr,
    wp::array_t<wp::vec_t<3, wp::float32>> var_mesh_vert,
    wp::array_t<wp::int32> var_mesh_graph,
    wp::array_t<wp::int32> var_mesh_polynum,
    wp::array_t<wp::int32> var_mesh_polyadr,
    wp::array_t<wp::vec_t<3, wp::float32>> var_mesh_polynormal,
    wp::array_t<wp::int32> var_mesh_polyvertadr,
    wp::array_t<wp::int32> var_mesh_polyvertnum,
    wp::array_t<wp::int32> var_mesh_polyvert,
    wp::array_t<wp::int32> var_mesh_polymapadr,
    wp::array_t<wp::int32> var_mesh_polymapnum,
    wp::array_t<wp::int32> var_mesh_polymap,
    wp::array_t<wp::int32> var_pair_dim,
    wp::array_t<wp::vec_t<2, wp::float32>> var_pair_solref,
    wp::array_t<wp::vec_t<2, wp::float32>> var_pair_solreffriction,
    wp::array_t<wp::vec_t<5, wp::float32>> var_pair_solimp,
    wp::array_t<wp::float32> var_pair_margin,
    wp::array_t<wp::float32> var_pair_gap,
    wp::array_t<wp::vec_t<5, wp::float32>> var_pair_friction,
    wp::array_t<wp::vec_t<3, wp::float32>> var_geom_xpos_in,
    wp::array_t<wp::mat_t<3, 3, wp::float32>> var_geom_xmat_in,
    wp::int32 var_naconmax_in,
    wp::array_t<wp::int32> var_ncollision_in,
    wp::array_t<wp::vec_t<2, wp::int32>> var_collision_pair_in,
    wp::array_t<wp::vec_t<2, wp::int32>> var_collision_pairid_in,
    wp::array_t<wp::int32> var_collision_worldid_in,
    wp::array_t<wp::float32> var_contact_dist_out,
    wp::array_t<wp::vec_t<3, wp::float32>> var_contact_pos_out,
    wp::array_t<wp::mat_t<3, 3, wp::float32>> var_contact_frame_out,
    wp::array_t<wp::float32> var_contact_includemargin_out,
    wp::array_t<wp::vec_t<5, wp::float32>> var_contact_friction_out,
    wp::array_t<wp::vec_t<2, wp::float32>> var_contact_solref_out,
    wp::array_t<wp::vec_t<2, wp::float32>> var_contact_solreffriction_out,
    wp::array_t<wp::vec_t<5, wp::float32>> var_contact_solimp_out,
    wp::array_t<wp::int32> var_contact_dim_out,
    wp::array_t<wp::vec_t<2, wp::int32>> var_contact_geom_out,
    wp::array_t<wp::int32> var_contact_efc_address_out,
    wp::array_t<wp::int32> var_contact_worldid_out,
    wp::array_t<wp::int32> var_contact_type_out,
    wp::array_t<wp::int32> var_contact_geomcollisionid_out,
    wp::array_t<wp::int32> var_nacon_out)
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
        wp::vec_t<2, wp::int32>* var_5;
        wp::vec_t<2, wp::int32> var_6;
        wp::vec_t<2, wp::int32> var_7;
        wp::int32* var_8;
        wp::int32 var_9;
        wp::int32 var_10;
        wp::vec_t<2, wp::int32> var_11;
        wp::float32 var_12;
        wp::float32 var_13;
        wp::int32 var_14;
        wp::vec_t<5, wp::float32> var_15;
        wp::vec_t<2, wp::float32> var_16;
        wp::vec_t<2, wp::float32> var_17;
        wp::vec_t<5, wp::float32> var_18;
        Geom_3242f8a8 var_19;
        Geom_3242f8a8 var_20;
        const wp::int32 var_21 = 0;
        const wp::int32 var_22 = 0;
        const wp::int32 var_23 = 5;
        const wp::int32 var_24 = 0;
        wp::int32 var_25;
        wp::int32* var_26;
        wp::int32 var_27;
        wp::int32 var_28;
        const wp::int32 var_29 = 1;
        wp::int32 var_30;
        wp::int32* var_31;
        wp::int32 var_32;
        wp::int32 var_33;
        bool var_34;
        bool var_35;
        bool var_36;
        wp::vec_t<2, wp::int32>* var_37;
        wp::vec_t<2, wp::int32> var_38;
        const wp::int32 var_39 = 1;
        const wp::int32 var_40 = 0;
        const wp::int32 var_41 = 7;
        const wp::int32 var_42 = 0;
        wp::int32 var_43;
        wp::int32* var_44;
        wp::int32 var_45;
        wp::int32 var_46;
        const wp::int32 var_47 = 1;
        wp::int32 var_48;
        wp::int32* var_49;
        wp::int32 var_50;
        wp::int32 var_51;
        bool var_52;
        bool var_53;
        bool var_54;
        wp::vec_t<2, wp::int32>* var_55;
        wp::vec_t<2, wp::int32> var_56;
        //---------
        // forward
        // def primitive_narrowphase(                                                             <L 1303>
        // tid = wp.tid()                                                                         <L 1363>
        var_0 = builtin_tid1d();
        // if tid >= ncollision_in[0]:                                                            <L 1365>
        var_2 = wp::address(var_ncollision_in, var_1);
        var_4 = wp::load(var_2);
        var_3 = (var_0 >= var_4);
        if (var_3) {
            // return                                                                             <L 1366>
            continue;
        }
        // geoms = collision_pair_in[tid]                                                         <L 1368>
        var_5 = wp::address(var_collision_pair_in, var_0);
        var_7 = wp::load(var_5);
        var_6 = wp::copy(var_7);
        // worldid = collision_worldid_in[tid]                                                    <L 1369>
        var_8 = wp::address(var_collision_worldid_in, var_0);
        var_10 = wp::load(var_8);
        var_9 = wp::copy(var_10);
        // _, margin, gap, condim, friction, solref, solreffriction, solimp = contact_params(       <L 1371>
        // geom_condim,                                                                           <L 1372>
        // geom_priority,                                                                         <L 1373>
        // geom_solmix,                                                                           <L 1374>
        // geom_solref,                                                                           <L 1375>
        // geom_solimp,                                                                           <L 1376>
        // geom_friction,                                                                         <L 1377>
        // geom_margin,                                                                           <L 1378>
        // geom_gap,                                                                              <L 1379>
        // pair_dim,                                                                              <L 1380>
        // pair_solref,                                                                           <L 1381>
        // pair_solreffriction,                                                                   <L 1382>
        // pair_solimp,                                                                           <L 1383>
        // pair_margin,                                                                           <L 1384>
        // pair_gap,                                                                              <L 1385>
        // pair_friction,                                                                         <L 1386>
        // collision_pair_in,                                                                     <L 1387>
        // collision_pairid_in,                                                                   <L 1388>
        // tid,                                                                                   <L 1389>
        // worldid,                                                                               <L 1390>
        contact_params_0(var_geom_condim, var_geom_priority, var_geom_solmix, var_geom_solref, var_geom_solimp, var_geom_friction, var_geom_margin, var_geom_gap, var_pair_dim, var_pair_solref, var_pair_solreffriction, var_pair_solimp, var_pair_margin, var_pair_gap, var_pair_friction, var_collision_pair_in, var_collision_pairid_in, var_0, var_9, var_11, var_12, var_13, var_14, var_15, var_16, var_17, var_18);
        // geom1, geom2 = geom_collision_pair(                                                    <L 1393>
        // geom_type,                                                                             <L 1394>
        // geom_dataid,                                                                           <L 1395>
        // geom_size,                                                                             <L 1396>
        // mesh_vertadr,                                                                          <L 1397>
        // mesh_vertnum,                                                                          <L 1398>
        // mesh_graphadr,                                                                         <L 1399>
        // mesh_vert,                                                                             <L 1400>
        // mesh_graph,                                                                            <L 1401>
        // mesh_polynum,                                                                          <L 1402>
        // mesh_polyadr,                                                                          <L 1403>
        // mesh_polynormal,                                                                       <L 1404>
        // mesh_polyvertadr,                                                                      <L 1405>
        // mesh_polyvertnum,                                                                      <L 1406>
        // mesh_polyvert,                                                                         <L 1407>
        // mesh_polymapadr,                                                                       <L 1408>
        // mesh_polymapnum,                                                                       <L 1409>
        // mesh_polymap,                                                                          <L 1410>
        // geom_xpos_in,                                                                          <L 1411>
        // geom_xmat_in,                                                                          <L 1412>
        // geoms,                                                                                 <L 1413>
        // worldid,                                                                               <L 1414>
        geom_collision_pair_0(var_geom_type, var_geom_dataid, var_geom_size, var_mesh_vertadr, var_mesh_vertnum, var_mesh_graphadr, var_mesh_vert, var_mesh_graph, var_mesh_polynum, var_mesh_polyadr, var_mesh_polynormal, var_mesh_polyvertadr, var_mesh_polyvertnum, var_mesh_polyvert, var_mesh_polymapadr, var_mesh_polymapnum, var_mesh_polymap, var_geom_xpos_in, var_geom_xmat_in, var_6, var_9, var_19, var_20);
        // for i in range(wp.static(len(primitive_collisions_func))):                             <L 1417>
        // collision_type1 = wp.static(primitive_collisions_types[i][0])                          <L 1418>
        // collision_type2 = wp.static(primitive_collisions_types[i][1])                          <L 1419>
        // type1 = geom_type[geoms[0]]                                                            <L 1420>
        var_25 = wp::extract(var_6, var_24);
        var_26 = wp::address(var_geom_type, var_25);
        var_28 = wp::load(var_26);
        var_27 = wp::copy(var_28);
        // type2 = geom_type[geoms[1]]                                                            <L 1421>
        var_30 = wp::extract(var_6, var_29);
        var_31 = wp::address(var_geom_type, var_30);
        var_33 = wp::load(var_31);
        var_32 = wp::copy(var_33);
        // if collision_type1 == type1 and collision_type2 == type2:                              <L 1422>
        var_35 = (var_22 == var_27);
        var_34 = var_35;
        if (var_34) {
            var_36 = (var_23 == var_32);
            var_34 = var_34 && var_36;
        }
        if (var_34) {
            // wp.static(primitive_collisions_func[i])(                                           <L 1423>
            // naconmax_in,                                                                       <L 1424>
            // geom1,                                                                             <L 1425>
            // geom2,                                                                             <L 1426>
            // worldid,                                                                           <L 1427>
            // margin,                                                                            <L 1428>
            // gap,                                                                               <L 1429>
            // condim,                                                                            <L 1430>
            // friction,                                                                          <L 1431>
            // solref,                                                                            <L 1432>
            // solreffriction,                                                                    <L 1433>
            // solimp,                                                                            <L 1434>
            // geoms,                                                                             <L 1435>
            // collision_pairid_in[tid],                                                          <L 1436>
            var_37 = wp::address(var_collision_pairid_in, var_0);
            // contact_dist_out,                                                                  <L 1437>
            // contact_pos_out,                                                                   <L 1438>
            // contact_frame_out,                                                                 <L 1439>
            // contact_includemargin_out,                                                         <L 1440>
            // contact_friction_out,                                                              <L 1441>
            // contact_solref_out,                                                                <L 1442>
            // contact_solreffriction_out,                                                        <L 1443>
            // contact_solimp_out,                                                                <L 1444>
            // contact_dim_out,                                                                   <L 1445>
            // contact_geom_out,                                                                  <L 1446>
            // contact_efc_address_out,                                                           <L 1447>
            // contact_worldid_out,                                                               <L 1448>
            // contact_type_out,                                                                  <L 1449>
            // contact_geomcollisionid_out,                                                       <L 1450>
            // nacon_out,                                                                         <L 1451>
            var_38 = wp::load(var_37);
            plane_cylinder_wrapper_0(var_naconmax_in, var_19, var_20, var_9, var_12, var_13, var_14, var_15, var_16, var_17, var_18, var_6, var_38, var_contact_dist_out, var_contact_pos_out, var_contact_frame_out, var_contact_includemargin_out, var_contact_friction_out, var_contact_solref_out, var_contact_solreffriction_out, var_contact_solimp_out, var_contact_dim_out, var_contact_geom_out, var_contact_efc_address_out, var_contact_worldid_out, var_contact_type_out, var_contact_geomcollisionid_out, var_nacon_out);
        }
        // collision_type1 = wp.static(primitive_collisions_types[i][0])                          <L 1418>
        // collision_type2 = wp.static(primitive_collisions_types[i][1])                          <L 1419>
        // type1 = geom_type[geoms[0]]                                                            <L 1420>
        var_43 = wp::extract(var_6, var_42);
        var_44 = wp::address(var_geom_type, var_43);
        var_46 = wp::load(var_44);
        var_45 = wp::copy(var_46);
        // type2 = geom_type[geoms[1]]                                                            <L 1421>
        var_48 = wp::extract(var_6, var_47);
        var_49 = wp::address(var_geom_type, var_48);
        var_51 = wp::load(var_49);
        var_50 = wp::copy(var_51);
        // if collision_type1 == type1 and collision_type2 == type2:                              <L 1422>
        var_53 = (var_40 == var_45);
        var_52 = var_53;
        if (var_52) {
            var_54 = (var_41 == var_50);
            var_52 = var_52 && var_54;
        }
        if (var_52) {
            // wp.static(primitive_collisions_func[i])(                                           <L 1423>
            // naconmax_in,                                                                       <L 1424>
            // geom1,                                                                             <L 1425>
            // geom2,                                                                             <L 1426>
            // worldid,                                                                           <L 1427>
            // margin,                                                                            <L 1428>
            // gap,                                                                               <L 1429>
            // condim,                                                                            <L 1430>
            // friction,                                                                          <L 1431>
            // solref,                                                                            <L 1432>
            // solreffriction,                                                                    <L 1433>
            // solimp,                                                                            <L 1434>
            // geoms,                                                                             <L 1435>
            // collision_pairid_in[tid],                                                          <L 1436>
            var_55 = wp::address(var_collision_pairid_in, var_0);
            // contact_dist_out,                                                                  <L 1437>
            // contact_pos_out,                                                                   <L 1438>
            // contact_frame_out,                                                                 <L 1439>
            // contact_includemargin_out,                                                         <L 1440>
            // contact_friction_out,                                                              <L 1441>
            // contact_solref_out,                                                                <L 1442>
            // contact_solreffriction_out,                                                        <L 1443>
            // contact_solimp_out,                                                                <L 1444>
            // contact_dim_out,                                                                   <L 1445>
            // contact_geom_out,                                                                  <L 1446>
            // contact_efc_address_out,                                                           <L 1447>
            // contact_worldid_out,                                                               <L 1448>
            // contact_type_out,                                                                  <L 1449>
            // contact_geomcollisionid_out,                                                       <L 1450>
            // nacon_out,                                                                         <L 1451>
            var_56 = wp::load(var_55);
            plane_convex_wrapper_0(var_naconmax_in, var_19, var_20, var_9, var_12, var_13, var_14, var_15, var_16, var_17, var_18, var_6, var_56, var_contact_dist_out, var_contact_pos_out, var_contact_frame_out, var_contact_includemargin_out, var_contact_friction_out, var_contact_solref_out, var_contact_solreffriction_out, var_contact_solimp_out, var_contact_dim_out, var_contact_geom_out, var_contact_efc_address_out, var_contact_worldid_out, var_contact_type_out, var_contact_geomcollisionid_out, var_nacon_out);
        }
    }
}


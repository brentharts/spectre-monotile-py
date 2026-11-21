// spectre_exp_ab.cpp - Fast OpenMP C++20 CNF generator for Experiments A and B:
//   Mode "multi_cnf" (Experiment B): Exact Stokes-flux + overlap SAT CNF for ARBITRARY
//       (possibly disconnected / multiply-connected / Swiss-cheese) erased tile subsets E
//       inside a Level-4 Spectre patch, checking candidate containment strictly by
//       non-overlap against nearby pinned tiles!
//   Mode "relax_cnf" (Experiment A): Exact Relaxed-SAT soft-edge-defect CNF with Sinz
//       sequential cardinality counter sum(d_e) <= D, forcing >= 1 NEW tile (non-trivial
//       chiral rearrangement!) and optional tile-count lower bound |T'| >= k_min.
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <map>
#include <set>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

static constexpr double SQRT3 = 1.7320508075688772935;
using i64 = int64_t;
using i128 = __int128_t;

struct Z3Num { i64 a, b; bool operator==(const Z3Num& o) const { return a==o.a && b==o.b; } };
struct Z3Pt { Z3Num x, y; bool operator==(const Z3Pt& o) const { return x==o.x && y==o.y; } };
struct Z4Vec {
    i64 v[4];
    bool operator==(const Z4Vec& o) const { return v[0]==o.v[0] && v[1]==o.v[1] && v[2]==o.v[2] && v[3]==o.v[3]; }
    bool operator<(const Z4Vec& o) const { for (int i=0;i<4;++i) if (v[i]!=o.v[i]) return v[i]<o.v[i]; return false; }
};
struct Z4Hash {
    size_t operator()(const Z4Vec& x) const noexcept {
        uint64_t h = 0x9e3779b97f4a7c15ULL;
        for (int i=0;i<4;++i) h ^= uint64_t(x.v[i]) + 0x9e3779b97f4a7c15ULL + (h<<6) + (h>>2);
        return size_t(h);
    }
};
struct DirEdge {
    Z4Vec u, v;
    bool operator==(const DirEdge& o) const { return u==o.u && v==o.v; }
    bool operator<(const DirEdge& o) const { return (u==o.u) ? (v<o.v) : (u<o.u); }
};
struct DirEdgeHash {
    size_t operator()(const DirEdge& e) const noexcept {
        Z4Hash zh; size_t h1 = zh(e.u), h2 = zh(e.v);
        return h1 ^ (h2 + 0x9e3779b97f4a7c15ULL + (h1<<6) + (h1>>2));
    }
};
struct PlacementKey {
    int orient; Z4Vec shift;
    bool operator==(const PlacementKey& o) const { return orient==o.orient && shift==o.shift; }
    bool operator<(const PlacementKey& o) const { return (orient!=o.orient)?(orient<o.orient):(shift<o.shift); }
};
struct PlacementKeyHash {
    size_t operator()(const PlacementKey& k) const noexcept {
        size_t h = Z4Hash{}(k.shift);
        return h ^ (size_t(k.orient)*0x9e3779b97f4a7c15ULL + (h<<6) + (h>>2));
    }
};

inline int z3_sign(i64 a, i64 b) {
    if (a==0 && b==0) return 0;
    if (a>=0 && b>=0) return 1;
    if (a<=0 && b<=0) return -1;
    i128 aa = i128(a)*i128(a), bb3 = i128(3)*i128(b)*i128(b);
    if (a>0) return (aa>bb3)?1:((aa<bb3)?-1:0);
    return (bb3>aa)?1:((bb3<aa)?-1:0);
}
inline int z3_cmp(Z3Num u, Z3Num v) { return z3_sign(u.a-v.a, u.b-v.b); }
inline int z3_orient(const Z3Pt& o, const Z3Pt& a, const Z3Pt& b) {
    i128 ax0=a.x.a-o.x.a, ax1=a.x.b-o.x.b, ay0=a.y.a-o.y.a, ay1=a.y.b-o.y.b;
    i128 bx0=b.x.a-o.x.a, bx1=b.x.b-o.x.b, by0=b.y.a-o.y.a, by1=b.y.b-o.y.b;
    i128 c0=(ax0*by0+3*ax1*by1)-(ay0*bx0+3*ay1*bx1);
    i128 c1=(ax0*by1+ax1*by0)-(ay0*bx1+ay1*bx0);
    if (c0==0 && c1==0) return 0;
    if (c0>=0 && c1>=0) return 1;
    if (c0<=0 && c1<=0) return -1;
    i128 lhs=c0*c0, rhs=i128(3)*c1*c1;
    if (c0>0) return (lhs>rhs)?1:((lhs<rhs)?-1:0);
    return (rhs>lhs)?1:((rhs<lhs)?-1:0);
}
inline bool z3_on_segment_strict(const Z3Pt& p, const Z3Pt& a, const Z3Pt& b) {
    if (p==a || p==b) return false;
    if (z3_orient(a,b,p)!=0) return false;
    return (z3_cmp(p.x,a.x)*z3_cmp(p.x,b.x)<=0) && (z3_cmp(p.y,a.y)*z3_cmp(p.y,b.y)<=0);
}
inline bool z3_segments_properly_cross(const Z3Pt& a, const Z3Pt& b, const Z3Pt& c, const Z3Pt& d) {
    int o1=z3_orient(a,b,c), o2=z3_orient(a,b,d);
    if (o1==0 || o2==0 || o1==o2) return false;
    int o3=z3_orient(c,d,a), o4=z3_orient(c,d,b);
    return o3!=0 && o4!=0 && o3!=o4;
}
inline Z3Pt z4_to_z3pt(const Z4Vec& v) { return {{2*v.v[0]+v.v[1], v.v[2]}, {v.v[2]+2*v.v[3], v.v[1]}}; }
inline std::pair<double,double> z4_to_f2(const Z4Vec& v) {
    return {v.v[0]+0.5*v.v[1]+0.5*SQRT3*v.v[2], 0.5*SQRT3*v.v[1]+0.5*v.v[2]+v.v[3]};
}
static int z3_point_in_poly(const Z3Pt& p, const std::vector<Z3Pt>& poly) {
    int n=int(poly.size()); bool inside=false;
    for (int i=0;i<n;++i) {
        const Z3Pt& a=poly[i], &b=poly[(i+1)%n];
        if (p==a || z3_on_segment_strict(p,a,b)) return 0;
        int ca=z3_cmp(a.y,p.y), cb=z3_cmp(b.y,p.y);
        if ((ca>0)!=(cb>0)) {
            if (cb>0) { if (z3_orient(a,b,p)>0) inside=!inside; }
            else { if (z3_orient(b,a,p)>0) inside=!inside; }
        }
    }
    return inside?1:-1;
}

struct TileGeom {
    PlacementKey key;
    std::array<Z4Vec,14> verts;
    std::array<DirEdge,14> edges;
    std::vector<Z3Pt> poly, poly2;
    double x0,y0,x1,y1;
};

static std::array<std::array<Z4Vec,14>,12> ORIENT_VERTS;
static void init_tables() {
    int MZ[4][4]={{0,0,0,-1},{0,0,1,1},{1,0,0,0},{0,1,0,0}};
    int ZP[12][4][4];
    for (int i=0;i<4;++i) for (int j=0;j<4;++j) ZP[0][i][j]=(i==j);
    for (int k=1;k<12;++k) for (int i=0;i<4;++i) for (int j=0;j<4;++j) {
        int s=0; for (int m=0;m<4;++m) s+=MZ[i][m]*ZP[k-1][m][j]; ZP[k][i][j]=s;
    }
    int edge_dirs[14]={0,10,1,3,0,2,5,7,4,6,6,8,11,9};
    std::array<Z4Vec,14> base; base[0]={{0,0,0,0}};
    for (int i=0;i<13;++i) for (int c=0;c<4;++c) base[i+1].v[c]=base[i].v[c]+ZP[edge_dirs[i]][c][0];
    for (int k=0;k<12;++k) for (int i=0;i<14;++i) {
        Z4Vec p{}; for (int r=0;r<4;++r) { i64 s=0; for (int c=0;c<4;++c) s+=ZP[k][r][c]*base[i].v[c]; p.v[r]=s; }
        ORIENT_VERTS[k][i]=p;
    }
}
static TileGeom make_tile(int o, const Z4Vec& sh) {
    TileGeom tg; tg.key={o,sh}; tg.poly.resize(14); tg.poly2.resize(14);
    tg.x0=tg.y0=1e18; tg.x1=tg.y1=-1e18;
    for (int i=0;i<14;++i) {
        for (int c=0;c<4;++c) tg.verts[i].v[c]=ORIENT_VERTS[o][i].v[c]+sh.v[c];
        tg.poly[i]=z4_to_z3pt(tg.verts[i]);
        tg.poly2[i]={{2*tg.poly[i].x.a,2*tg.poly[i].x.b},{2*tg.poly[i].y.a,2*tg.poly[i].y.b}};
        auto [fx,fy]=z4_to_f2(tg.verts[i]);
        tg.x0=std::min(tg.x0,fx); tg.y0=std::min(tg.y0,fy); tg.x1=std::max(tg.x1,fx); tg.y1=std::max(tg.y1,fy);
    }
    for (int i=0;i<14;++i) tg.edges[i]={tg.verts[i],tg.verts[(i+1)%14]};
    return tg;
}
static bool has_pt(const std::vector<Z3Pt>& poly, const Z3Pt& p) {
    for (const auto& q : poly) if (q==p) return true; return false;
}
static bool tiles_overlap_exact(const TileGeom& A, const TileGeom& B) {
    if (A.x1<B.x0-1e-6 || B.x1<A.x0-1e-6 || A.y1<B.y0-1e-6 || B.y1<A.y0-1e-6) return false;
    bool same=true; for (int i=0;i<14;++i) if (!has_pt(B.poly,A.poly[i])) { same=false; break; }
    if (same) return true;
    for (int i=0;i<14;++i) for (int j=0;j<14;++j)
        if (z3_segments_properly_cross(A.poly[i],A.poly[(i+1)%14],B.poly[j],B.poly[(j+1)%14])) return true;
    for (int i=0;i<14;++i) if (!has_pt(B.poly,A.poly[i]) && z3_point_in_poly(A.poly[i],B.poly)==1) return true;
    for (int j=0;j<14;++j) if (!has_pt(A.poly,B.poly[j]) && z3_point_in_poly(B.poly[j],A.poly)==1) return true;
    std::vector<Z3Pt> sp; sp.reserve(6);
    auto chk = [&](const TileGeom& P, const TileGeom& Q) {
        for (int i=0;i<14;++i) {
            const Z3Pt& a1=P.poly[i], &a2=P.poly[(i+1)%14];
            sp.clear(); sp.push_back(a1); sp.push_back(a2);
            for (int j=0;j<14;++j) if (z3_on_segment_strict(Q.poly[j],a1,a2)) sp.push_back(Q.poly[j]);
            if (sp.size()>2) {
                double dx=(a2.x.a-a1.x.a)+(a2.x.b-a1.x.b)*SQRT3, dy=(a2.y.a-a1.y.a)+(a2.y.b-a1.y.b)*SQRT3;
                std::sort(sp.begin(),sp.end(),[&](const Z3Pt& u, const Z3Pt& v){
                    return ((u.x.a+u.x.b*SQRT3)*dx+(u.y.a+u.y.b*SQRT3)*dy) < ((v.x.a+v.x.b*SQRT3)*dx+(v.y.a+v.y.b*SQRT3)*dy);
                });
            }
            for (size_t s=0;s+1<sp.size();++s) {
                Z3Pt mid2{{sp[s].x.a+sp[s+1].x.a,sp[s].x.b+sp[s+1].x.b},{sp[s].y.a+sp[s+1].y.a,sp[s].y.b+sp[s+1].y.b}};
                if (z3_point_in_poly(mid2,Q.poly2)==1) return true;
            }
        }
        return false;
    };
    return chk(A,B) || chk(B,A);
}

// Sinz sequential counter: sum(lits) <= K (introduces auxiliary vars starting at next_var)
static void add_sinz_at_most_k(const std::vector<int>& lits, int K, int& next_var, std::vector<std::vector<int>>& clauses) {
    int n = int(lits.size());
    if (K >= n) return;
    if (K == 0) {
        for (int l : lits) clauses.push_back({-l});
        return;
    }
    // s[i][j] for i in 0..n-2, j in 0..K-1 (meaning sum_{t<=i} x_t >= j+1)
    std::vector<std::vector<int>> s(n - 1, std::vector<int>(K));
    for (int i=0;i<n-1;++i) for (int j=0;j<K;++j) s[i][j] = next_var++;
    // i = 0
    clauses.push_back({-lits[0], s[0][0]});
    for (int j=1;j<K;++j) clauses.push_back({-s[0][j]});
    // 1 <= i <= n-2
    for (int i=1;i<n-1;++i) {
        clauses.push_back({-lits[i], s[i][0]});
        clauses.push_back({-s[i-1][0], s[i][0]});
        for (int j=1;j<K;++j) {
            clauses.push_back({-lits[i], -s[i-1][j-1], s[i][j]});
            clauses.push_back({-s[i-1][j], s[i][j]});
        }
        clauses.push_back({-lits[i], -s[i-1][K-1]});
    }
    // i = n-1
    clauses.push_back({-lits[n-1], -s[n-2][K-1]});
}

// Read pinned exterior tiles + original erased tiles + raw candidate placements;
// filter candidates strictly non-overlapping with ALL pinned exterior tiles!
static void load_and_filter(
    std::vector<TileGeom>& orig,
    std::vector<TileGeom>& pinned,
    std::vector<TileGeom>& cands,
    std::unordered_set<DirEdge, DirEdgeHash>& bd_edges,
    std::unordered_map<PlacementKey, int, PlacementKeyHash>& key_to_var
) {
    int n_orig, n_pin, n_raw;
    std::cin >> n_orig >> n_pin >> n_raw;
    orig.reserve(n_orig);
    std::unordered_map<DirEdge, int, DirEdgeHash> ecnt;
    double ex0=1e18, ey0=1e18, ex1=-1e18, ey1=-1e18;
    for (int i=0;i<n_orig;++i) {
        int o; Z4Vec sh; std::cin >> o >> sh.v[0] >> sh.v[1] >> sh.v[2] >> sh.v[3];
        TileGeom tg = make_tile(o, sh);
        ex0=std::min(ex0,tg.x0); ey0=std::min(ey0,tg.y0); ex1=std::max(ex1,tg.x1); ey1=std::max(ey1,tg.y1);
        for (int j=0;j<14;++j) ecnt[tg.edges[j]]++;
        orig.push_back(std::move(tg));
    }
    for (const auto& [e, c] : ecnt) if (!ecnt.count({e.v, e.u})) bd_edges.insert(e);

    pinned.reserve(n_pin);
    std::map<std::pair<int,int>, std::vector<int>> pin_buckets;
    for (int i=0;i<n_pin;++i) {
        int o; Z4Vec sh; std::cin >> o >> sh.v[0] >> sh.v[1] >> sh.v[2] >> sh.v[3];
        TileGeom tg = make_tile(o, sh);
        int bx0=int(std::floor(tg.x0/4.0)), bx1=int(std::floor(tg.x1/4.0));
        int by0=int(std::floor(tg.y0/4.0)), by1=int(std::floor(tg.y1/4.0));
        for (int bx=bx0;bx<=bx1;++bx) for (int by=by0;by<=by1;++by) pin_buckets[{bx,by}].push_back(i);
        pinned.push_back(std::move(tg));
    }

    std::vector<PlacementKey> raw(n_raw);
    for (int i=0;i<n_raw;++i)
        std::cin >> raw[i].orient >> raw[i].shift.v[0] >> raw[i].shift.v[1] >> raw[i].shift.v[2] >> raw[i].shift.v[3];

    std::vector<uint8_t> keep(n_raw, 0);
    std::vector<TileGeom> all_tg(n_raw);
    #pragma omp parallel for schedule(dynamic, 64)
    for (int i=0;i<n_raw;++i) {
        TileGeom tg = make_tile(raw[i].orient, raw[i].shift);
        // Must stay inside erased bounding box (guaranteed strictly inside pinned collar!)
        if (tg.x0 < ex0 - 1e-6 || tg.y0 < ey0 - 1e-6 || tg.x1 > ex1 + 1e-6 || tg.y1 > ey1 + 1e-6) continue;
        // Must NOT overlap ANY pinned exterior tile!
        bool bad = false;
        int bx0=int(std::floor(tg.x0/4.0)), bx1=int(std::floor(tg.x1/4.0));
        int by0=int(std::floor(tg.y0/4.0)), by1=int(std::floor(tg.y1/4.0));
        for (int bx=bx0; bx<=bx1 && !bad; ++bx) {
            for (int by=by0; by<=by1 && !bad; ++by) {
                auto it = pin_buckets.find({bx, by});
                if (it == pin_buckets.end()) continue;
                for (int pid : it->second) {
                    if (tiles_overlap_exact(tg, pinned[pid])) { bad = true; break; }
                }
            }
        }
        if (!bad) { keep[i]=1; all_tg[i]=std::move(tg); }
    }
    for (int i=0;i<n_raw;++i) if (keep[i]) {
        int vid = int(cands.size()) + 1;
        key_to_var[all_tg[i].key] = vid;
        cands.push_back(std::move(all_tg[i]));
    }
}

static std::vector<std::pair<int,int>> find_overlaps(const std::vector<TileGeom>& cands) {
    int M = int(cands.size());
    std::map<std::pair<int,int>, std::vector<int>> buckets;
    for (int i=0;i<M;++i) {
        int bx0=int(std::floor(cands[i].x0/4.0)), bx1=int(std::floor(cands[i].x1/4.0));
        int by0=int(std::floor(cands[i].y0/4.0)), by1=int(std::floor(cands[i].y1/4.0));
        for (int bx=bx0;bx<=bx1;++bx) for (int by=by0;by<=by1;++by) buckets[{bx,by}].push_back(i);
    }
    std::vector<std::pair<int,int>> plist;
    std::unordered_set<uint64_t> seen;
    seen.reserve(size_t(M)*32);
    for (const auto& [_, ids] : buckets) {
        for (size_t a=0;a<ids.size();++a) for (size_t b=a+1;b<ids.size();++b) {
            int ia=std::min(ids[a],ids[b]), ib=std::max(ids[a],ids[b]);
            if (seen.insert((uint64_t(ia)<<32)|uint64_t(ib)).second) plist.push_back({ia,ib});
        }
    }
    std::vector<uint8_t> ov(plist.size(), 0);
    #pragma omp parallel for schedule(dynamic, 1024)
    for (size_t p=0;p<plist.size();++p) if (tiles_overlap_exact(cands[plist[p].first], cands[plist[p].second])) ov[p]=1;
    std::vector<std::pair<int,int>> out;
    for (size_t p=0;p<plist.size();++p) if (ov[p]) out.push_back(plist[p]);
    return out;
}

static int run_multi_cnf() {
    std::string cnf_path; int forbid_orig;
    if (!(std::cin >> cnf_path >> forbid_orig)) return 1;
    std::vector<TileGeom> orig, pinned, cands;
    std::unordered_set<DirEdge, DirEdgeHash> bd_edges;
    std::unordered_map<PlacementKey, int, PlacementKeyHash> key_to_var;
    load_and_filter(orig, pinned, cands, bd_edges, key_to_var);
    int M = int(cands.size());
    std::unordered_map<DirEdge, std::vector<int>, DirEdgeHash> edge_vars;
    for (int i=0;i<M;++i) for (int j=0;j<14;++j) edge_vars[cands[i].edges[j]].push_back(i+1);

    std::vector<std::vector<int>> clauses;
    for (const auto& e : bd_edges) {
        const auto& vs = edge_vars[e];
        clauses.push_back(vs);
        for (size_t i=0;i<vs.size();++i) for (size_t j=i+1;j<vs.size();++j) clauses.push_back({-vs[i],-vs[j]});
    }
    for (const auto& [e, vs] : edge_vars) {
        if (bd_edges.count(e)) continue;
        for (size_t i=0;i<vs.size();++i) for (size_t j=i+1;j<vs.size();++j) clauses.push_back({-vs[i],-vs[j]});
        auto oit = edge_vars.find({e.v, e.u});
        for (int v : vs) {
            std::vector<int> cl{-v};
            if (oit != edge_vars.end()) for (int ov : oit->second) cl.push_back(ov);
            clauses.push_back(std::move(cl));
        }
    }
    auto ovs = find_overlaps(cands);
    for (auto [a,b] : ovs) clauses.push_back({-(a+1), -(b+1)});
    if (forbid_orig) {
        std::vector<int> blk;
        for (const auto& tg : orig) {
            auto it = key_to_var.find(tg.key);
            if (it == key_to_var.end()) return 3;
            blk.push_back(-it->second);
        }
        clauses.push_back(std::move(blk));
    }
    std::ofstream out(cnf_path);
    out << "p cnf " << M << " " << clauses.size() << "\n";
    for (const auto& cl : clauses) { for (int l : cl) out << l << " "; out << "0\n"; }
    std::cout << "OK " << M << " " << clauses.size() << " " << ovs.size() << "\n";
    return 0;
}

// Mode "relax_cnf": outputs a reusable base CNF + defect-indicator var list, OR directly emits
// Sinz at-most-D over ALL unmatched directed edges (boundary + interior), requiring >= 1 NEW tile
// and (if conserve_area==1) requiring sum(x_t) >= n_orig (which together with non-overlap inside C
// forces exact area conservation |T'| == n_orig!).
static int run_relax_cnf() {
    std::string cnf_path; int max_D, conserve_area;
    if (!(std::cin >> cnf_path >> max_D >> conserve_area)) return 1;
    std::vector<TileGeom> orig, pinned, cands;
    std::unordered_set<DirEdge, DirEdgeHash> bd_edges;
    std::unordered_map<PlacementKey, int, PlacementKeyHash> key_to_var;
    load_and_filter(orig, pinned, cands, bd_edges, key_to_var);
    int M = int(cands.size());
    std::unordered_set<PlacementKey, PlacementKeyHash> orig_set;
    for (const auto& tg : orig) orig_set.insert(tg.key);

    std::unordered_map<DirEdge, std::vector<int>, DirEdgeHash> edge_vars;
    for (int i=0;i<M;++i) for (int j=0;j<14;++j) edge_vars[cands[i].edges[j]].push_back(i+1);

    int next_var = M + 1;
    std::vector<std::vector<int>> clauses;

    // 1. Pairwise overlap exclusion (strict!)
    auto ovs = find_overlaps(cands);
    for (auto [a,b] : ovs) clauses.push_back({-(a+1), -(b+1)});

    // 2. Defect indicator variable d_e for EVERY directed edge e (boundary or interior):
    //    - If e in bd_edges: d_e = 1 if NO chosen tile covers e (uncovered boundary edge!).
    //    - If e not in bd_edges: d_e = 1 if SOME chosen tile covers e AND NO chosen tile covers rev(e)
    //      (unmatched exposed interior edge!).
    std::vector<int> defect_vars;
    std::vector<std::pair<int, DirEdge>> defect_meta;
    for (const auto& e : bd_edges) {
        const auto& vs = edge_vars[e];
        for (size_t i=0;i<vs.size();++i) for (size_t j=i+1;j<vs.size();++j) clauses.push_back({-vs[i],-vs[j]});
        int dv = next_var++;
        defect_vars.push_back(dv);
        defect_meta.push_back({dv, e});
        std::vector<int> cl = vs;
        cl.push_back(dv); // (OR_{t ni e} x_t) OR d_e
        clauses.push_back(std::move(cl));
    }
    for (const auto& [e, vs] : edge_vars) {
        if (bd_edges.count(e)) continue;
        for (size_t i=0;i<vs.size();++i) for (size_t j=i+1;j<vs.size();++j) clauses.push_back({-vs[i],-vs[j]});
        int dv = next_var++;
        defect_vars.push_back(dv);
        defect_meta.push_back({dv, e});
        auto oit = edge_vars.find({e.v, e.u});
        for (int v : vs) {
            std::vector<int> cl{-v, dv};
            if (oit != edge_vars.end()) for (int ov : oit->second) cl.push_back(ov);
            clauses.push_back(std::move(cl));
        }
    }

    // 3. Non-triviality: at least 1 NEW tile (not in original tiling!) must be placed!
    std::vector<int> new_tile_lits;
    for (int i=0;i<M;++i) if (!orig_set.count(cands[i].key)) new_tile_lits.push_back(i + 1);
    clauses.push_back(new_tile_lits);

    // 4. Optional area conservation: sum_{i=1..M} x_i >= n_orig  <=>  sum (-x_i) <= M - n_orig?
    //    Wait: even better! Since every chosen tile is inside C and non-overlapping, |T'| <= n_orig always!
    //    Actually, for small cavities or if we want |T'| == n_orig, let's support conserve_area via
    //    allowing at most `max_missing` missing tiles? No: M can be 5000, so sum(-x_i) <= M-k is huge!
    //    Wait: we DON'T even need a giant cardinality counter for |T'|! Remember:
    //      For ANY packing T' of |T'| tiles inside C, the total exposed unmatched perimeter is:
    //        D(T') = |bd_edges| + 14*|T'| - 2*|matched_edges|
    //    Even simpler: to force ZERO missing area (|T'| == n_orig), every tile has area A_0, and
    //    any missing area creates a hole of perimeter >= 14!  In fact, if D_min < 14, then |T'| MUST
    //    equal n_orig automatically (no whole tile can be missing with perimeter < 14)!
    //    Wait — let's check both! If conserve_area == 1, we add a Sinz bound on UNCOVERED boundary
    //    edges <= max_D (already bounded by total defect <= max_D!).
    add_sinz_at_most_k(defect_vars, max_D, next_var, clauses);

    int total_vars = next_var - 1;
    std::ofstream out(cnf_path);
    out << "p cnf " << total_vars << " " << clauses.size() << "\n";
    for (const auto& cl : clauses) { for (int l : cl) out << l << " "; out << "0\n"; }

    std::cout << "OK " << M << " " << total_vars << " " << clauses.size() << " " << defect_vars.size() << " " << bd_edges.size() << "\n";
    for (int i=0;i<M;++i) {
        const auto& pk = cands[i].key;
        std::cout << pk.orient << " " << pk.shift.v[0] << " " << pk.shift.v[1] << " " << pk.shift.v[2] << " " << pk.shift.v[3]
                  << " " << (orig_set.count(pk) ? 1 : 0) << "\n";
    }
    return 0;
}

int main() {
    std::ios::sync_with_stdio(false);
    std::cin.tie(nullptr);
    init_tables();
    std::string mode;
    if (!(std::cin >> mode)) return 1;
    if (mode == "multi_cnf") return run_multi_cnf();
    if (mode == "relax_cnf") return run_relax_cnf();
    return 2;
}

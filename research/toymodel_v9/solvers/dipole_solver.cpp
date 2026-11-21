// dipole_solver.cpp - Fast exact Gray-code / OpenMP + multi-start SA/2-opt solver
// for the Spectre frustrated dipolar Ising model (quantgrav.pdf Result 17 / Open Problem #15).
// Input via stdin:
//   mode N (mode: "exact" or "sa", followed by N*N doubles W_{ij})
// Output:
//   E_min total_abs_W frac_unsat degeneracy spin_string
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <limits>
#include <random>
#include <string>
#include <vector>

#ifdef _OPENMP
#include <omp.h>
#endif

static double eval_energy(int N, const std::vector<double>& W, const std::vector<int8_t>& s) {
    double E = 0.0;
    for (int i = 0; i < N; ++i) {
        double acc = 0.0;
        const double* row = &W[static_cast<size_t>(i) * N];
        for (int j = i + 1; j < N; ++j) acc += row[j] * s[j];
        E += s[i] * acc;
    }
    return E;
}

static void solve_exact(int N, const std::vector<double>& W, double tot_abs) {
    // Fix s[N-1] = +1 by Z2 symmetry. Enumerate s[0..N-2] via partitioned Gray code across OpenMP threads!
    int M = N - 1;
    uint64_t total = uint64_t(1) << M;
    int prefix_bits = std::min(M, 8);
    uint64_t n_chunks = uint64_t(1) << prefix_bits;
    uint64_t chunk_sz = total >> prefix_bits;

    double global_best = std::numeric_limits<double>::infinity();
    uint64_t global_deg = 0;
    std::vector<int8_t> global_s(N, 1);

    #pragma omp parallel
    {
        double best = std::numeric_limits<double>::infinity();
        uint64_t deg = 0;
        std::vector<int8_t> best_s(N, 1);
        std::vector<int8_t> s(N, 1);
        std::vector<double> h(N, 0.0);

        #pragma omp for schedule(dynamic, 1)
        for (uint64_t c = 0; c < n_chunks; ++c) {
            uint64_t start_idx = c * chunk_sz;
            uint64_t g = start_idx ^ (start_idx >> 1);
            for (int i = 0; i < M; ++i) s[i] = ((g >> i) & 1ULL) ? -1 : 1;
            s[M] = 1;
            // Compute initial local fields h[i] = sum_{j != i} W_{ij} s_j and E
            double E = 0.0;
            for (int i = 0; i < N; ++i) {
                double acc = 0.0;
                const double* row = &W[static_cast<size_t>(i) * N];
                for (int j = 0; j < N; ++j) acc += row[j] * s[j];
                h[i] = acc;
                E += s[i] * acc;
            }
            E *= 0.5;

            auto record = [&](double curE) {
                if (curE < best - 1e-11) {
                    best = curE;
                    deg = 1;
                    best_s = s;
                } else if (std::abs(curE - best) <= 1e-11) {
                    ++deg;
                }
            };
            record(E);

            uint64_t prev_g = g;
            for (uint64_t step = 1; step < chunk_sz; ++step) {
                uint64_t idx = start_idx + step;
                uint64_t cur_g = idx ^ (idx >> 1);
                uint64_t diff = cur_g ^ prev_g;
                int bit = __builtin_ctzll(diff);
                prev_g = cur_g;
                // Flip s[bit]: deltaE = -2 * s[bit] * h[bit]
                int8_t old_si = s[bit];
                E -= 2.0 * old_si * h[bit];
                s[bit] = -old_si;
                double ds = -2.0 * old_si;
                const double* row = &W[static_cast<size_t>(bit) * N];
                for (int j = 0; j < N; ++j) h[j] += ds * row[j];
                record(E);
            }
        }

        #pragma omp critical
        {
            if (best < global_best - 1e-11) {
                global_best = best;
                global_deg = deg;
                global_s = best_s;
            } else if (std::abs(best - global_best) <= 1e-11) {
                global_deg += deg;
            }
        }
    }

    // Recompute exact energy to eliminate any floating drift
    double exact_E = eval_energy(N, W, global_s);
    double frac = (exact_E + tot_abs) / (2.0 * tot_abs);
    std::cout << std::setprecision(15) << exact_E << " " << tot_abs << " " << frac << " " << global_deg << " ";
    for (int i = 0; i < N; ++i) std::cout << (global_s[i] > 0 ? '+' : '-');
    std::cout << "\n";
}

static void solve_sa(int N, const std::vector<double>& W, double tot_abs, int n_restarts, int sweeps) {
    double global_best = std::numeric_limits<double>::infinity();
    int hits_best = 0;
    std::vector<int8_t> global_s(N, 1);

    // Precompute typical local field scale for temperature schedule
    double mean_h = 0.0;
    for (int i = 0; i < N; ++i) {
        double a = 0.0;
        for (int j = 0; j < N; ++j) a += std::abs(W[static_cast<size_t>(i) * N + j]);
        mean_h += a;
    }
    mean_h /= std::max(1, N);
    double T_hi = std::max(0.5, mean_h * 1.5);
    double T_lo = std::max(1e-5, mean_h * 1e-4);
    double ratio = std::pow(T_lo / T_hi, 1.0 / std::max(1, sweeps - 1));

    #pragma omp parallel
    {
        double best = std::numeric_limits<double>::infinity();
        int hits = 0;
        std::vector<int8_t> best_s(N, 1);

        #pragma omp for schedule(dynamic, 1)
        for (int r = 0; r < n_restarts; ++r) {
            std::mt19937_64 rng(0x9e3779b97f4a7c15ULL ^ (uint64_t(r + 1) * 0xbf58476d1ce4e5b9ULL));
            std::uniform_real_distribution<double> U(0.0, 1.0);
            std::vector<int8_t> s(N, 1);
            for (int i = 0; i < N - 1; ++i) s[i] = (rng() & 1ULL) ? 1 : -1;
            std::vector<double> h(N, 0.0);
            double E = 0.0;
            for (int i = 0; i < N; ++i) {
                double acc = 0.0;
                const double* row = &W[static_cast<size_t>(i) * N];
                for (int j = 0; j < N; ++j) acc += row[j] * s[j];
                h[i] = acc;
                E += s[i] * acc;
            }
            E *= 0.5;

            double T = T_hi;
            for (int sw = 0; sw < sweeps; ++sw) {
                for (int i = 0; i < N; ++i) {
                    double dE = -2.0 * s[i] * h[i];
                    if (dE <= 0.0 || U(rng) < std::exp(-dE / T)) {
                        int8_t old_si = s[i];
                        s[i] = -old_si;
                        E += dE;
                        double ds = -2.0 * old_si;
                        const double* row = &W[static_cast<size_t>(i) * N];
                        for (int j = 0; j < N; ++j) h[j] += ds * row[j];
                    }
                }
                T *= ratio;
            }

            // Greedy 1-opt + 2-opt polish to local minimum!
            bool improved = true;
            while (improved) {
                improved = false;
                for (int i = 0; i < N; ++i) {
                    double dE = -2.0 * s[i] * h[i];
                    if (dE < -1e-12) {
                        int8_t old_si = s[i];
                        s[i] = -old_si;
                        E += dE;
                        double ds = -2.0 * old_si;
                        const double* row = &W[static_cast<size_t>(i) * N];
                        for (int j = 0; j < N; ++j) h[j] += ds * row[j];
                        improved = true;
                    }
                }
                if (!improved && N <= 250) {
                    // 2-opt simultaneous pair flips
                    for (int i = 0; i < N && !improved; ++i) {
                        const double* rowi = &W[static_cast<size_t>(i) * N];
                        double dEi = -2.0 * s[i] * h[i];
                        for (int j = i + 1; j < N; ++j) {
                            double dEij = dEi - 2.0 * s[j] * h[j] + 4.0 * s[i] * s[j] * rowi[j];
                            if (dEij < -1e-12) {
                                int8_t oi = s[i], oj = s[j];
                                s[i] = -oi; s[j] = -oj;
                                E += dEij;
                                double dsi = -2.0 * oi, dsj = -2.0 * oj;
                                const double* rowj = &W[static_cast<size_t>(j) * N];
                                for (int k = 0; k < N; ++k) h[k] += dsi * rowi[k] + dsj * rowj[k];
                                improved = true;
                                break;
                            }
                        }
                    }
                }
            }

            double curE = eval_energy(N, W, s);
            if (curE < best - 1e-10) {
                best = curE;
                hits = 1;
                best_s = s;
            } else if (std::abs(curE - best) <= 1e-10) {
                ++hits;
            }
        }

        #pragma omp critical
        {
            if (best < global_best - 1e-10) {
                global_best = best;
                hits_best = hits;
                global_s = best_s;
            } else if (std::abs(best - global_best) <= 1e-10) {
                hits_best += hits;
            }
        }
    }

    if (global_s[N - 1] < 0) for (int i = 0; i < N; ++i) global_s[i] = -global_s[i];
    double exact_E = eval_energy(N, W, global_s);
    double frac = (exact_E + tot_abs) / (2.0 * tot_abs);
    std::cout << std::setprecision(15) << exact_E << " " << tot_abs << " " << frac << " " << hits_best << " ";
    for (int i = 0; i < N; ++i) std::cout << (global_s[i] > 0 ? '+' : '-');
    std::cout << "\n";
}

int main() {
    std::ios::sync_with_stdio(false);
    std::cin.tie(nullptr);
    std::string mode;
    int N = 0;
    if (!(std::cin >> mode >> N)) return 1;
    std::vector<double> W(static_cast<size_t>(N) * N, 0.0);
    double tot_abs = 0.0;
    for (int i = 0; i < N; ++i) {
        for (int j = 0; j < N; ++j) {
            std::cin >> W[static_cast<size_t>(i) * N + j];
            if (j > i) tot_abs += std::abs(W[static_cast<size_t>(i) * N + j]);
        }
    }
    if (mode == "exact") {
        solve_exact(N, W, tot_abs);
    } else if (mode == "sa") {
        int restarts = 512, sweeps = 400;
        std::cin >> restarts >> sweeps;
        solve_sa(N, W, tot_abs, restarts, sweeps);
    } else {
        return 2;
    }
    return 0;
}

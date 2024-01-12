#include <stdlib.h>
#include <iostream>
#include <functional>
#include <type_traits>
#include <random>
#include <array>
#include <map>
#include <unordered_map>
#include <omp.h>

#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>
#include <Eigen/Cholesky>

#include "arc_utilities/eigen_helpers.hpp"


#ifndef ARC_HELPERS_HPP
#define ARC_HELPERS_HPP

// Branch prediction hints
// Figure out which compiler we have
#if defined(__clang__)
    /* Clang/LLVM */
    #define likely(x) __builtin_expect(!!(x), 1)
    #define unlikely(x) __builtin_expect(!!(x), 0)
#elif defined(__ICC) || defined(__INTEL_COMPILER)
    /* Intel ICC/ICPC */
    #define likely(x) __builtin_expect(!!(x), 1)
    #define unlikely(x) __builtin_expect(!!(x), 0)
#elif defined(__GNUC__) || defined(__GNUG__)
    /* GNU GCC/G++ */
    #define likely(x) __builtin_expect(!!(x), 1)
    #define unlikely(x) __builtin_expect(!!(x), 0)
#elif defined(_MSC_VER)
    /* Microsoft Visual Studio */
    /* MSVC doesn't support branch prediction hints. Use PGO instead. */
    #define likely(x) (x)
    #define unlikely(x) (x)
#endif

// Macro to disable unused parameter compiler warnings
#define UNUSED(x) (void)(x)

namespace arc_helpers
{
    template<typename T>
    inline bool CheckAlignment(const T& item, const uint64_t desired_alignment)
    {
        const T* item_ptr = &item;
        const uintptr_t item_ptr_val = (uintptr_t)item_ptr;
        if ((item_ptr_val % desired_alignment) == 0)
        {
            //std::cout << "Item @ " << item_ptr_val << " aligned to " << desired_alignment << " bytes" << std::endl;
            return true;
        }
        else
        {
            //std::cout << "Item @ " << item_ptr_val << " not aligned to " << desired_alignment << " bytes" << std::endl;
            return false;
        }
    }

    template<typename T>
    inline void RequireAlignment(const T& item, const uint64_t desired_alignment)
    {
        if (CheckAlignment(item, desired_alignment) == false)
        {
            std::cout << "Item not aligned at desired alignment of " << desired_alignment << std::endl;
            assert(false);
        }
    }

    template <typename T>
    inline T SetBit(const T current, const uint32_t bit_position, const bool bit_value)
    {
        // Safety check on the type we've been called with
        static_assert((std::is_same<T, uint8_t>::value
                       || std::is_same<T, uint16_t>::value
                       || std::is_same<T, uint32_t>::value
                       || std::is_same<T, uint64_t>::value),
                      "Type must be a fixed-size unsigned integral type");
        // Do it
        T update_mask = 1;
        update_mask = update_mask << bit_position;
        if (bit_value)
        {
            return (current | update_mask);
        }
        else
        {
            update_mask = (~update_mask);
            return (current & update_mask);
        }
    }

    template <typename T>
    inline bool GetBit(const T current, const uint32_t bit_position)
    {
        // Type safety checks are performed in the SetBit() function
        const uint32_t mask = arc_helpers::SetBit((T)0, bit_position, true);
        if ((mask & current) > 0)
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    template <typename Key, typename Value, typename Compare=std::less<Key>, typename Allocator=std::allocator<std::pair<const Key, Value>>>
    inline Value RetrieveOrDefault(const std::map<Key, Value, Compare, Allocator>& map, const Key& key, const Value& default_val)
    {
        const auto found_itr = map.find(key);
        if (found_itr != map.end())
        {
            return found_itr->second;
        }
        else
        {
            return default_val;
        }
    }

    template <typename Key, typename Value, typename Hash=std::hash<Key>, typename Predicate=std::equal_to<Key>, typename Allocator=std::allocator<std::pair<const Key, Value>>>
    inline Value RetrieveOrDefault(const std::unordered_map<Key, Value, Hash, Predicate, Allocator>& map, const Key& key, const Value& default_val)
    {
        const auto found_itr = map.find(key);
        if (found_itr != map.end())
        {
            return found_itr->second;
        }
        else
        {
            return default_val;
        }
    }

    template <class T>
    inline T ClampValue(const T& val, const T& min, const T& max)
    {
        assert(min <= max);
        return std::min(max, std::max(min, val));
    }

    template <class T>
    inline T ClampValueAndWarn(const T& val, const T& min, const T& max)
    {
        assert(min <= max);
        if (val < min)
        {
            const std::string msg = "Clamping " + std::to_string(val) + " to min " + std::to_string(min) + "\n";
            std::cerr << msg << std::flush;
            return min;
        }
        else if (val > max)
        {
            const std::string msg = "Clamping " + std::to_string(val) + " to max " + std::to_string(max) + "\n";
            std::cerr << msg << std::flush;
            return max;
        }
        return val;
    }

    // Written to mimic parts of Matlab wthresh(val, 'h', thresh) behavior, spreading the value to teh thresholds instead of setting them to zero
    // https://www.mathworks.com/help/wavelet/ref/wthresh.html
    template <class T>
    inline T SpreadValue(const T& val, const T& low_threshold, const T& midpoint, const T& high_threshold)
    {
        assert(low_threshold <= midpoint);
        assert(midpoint <= high_threshold);
        if (val >= midpoint && val < high_threshold)
        {
            return high_threshold;
        }
        else if (val < midpoint && val > low_threshold)
        {
            return low_threshold;
        }
        return val;
    }

    // Written to mimic parts of Matlab wthresh(val, 'h', thresh) behavior, spreading the value to teh thresholds instead of setting them to zero
    // https://www.mathworks.com/help/wavelet/ref/wthresh.html
    template <class T>
    inline T SpreadValueAndWarn(const T& val, const T& low_threshold, const T& midpoint, const T& high_threshold)
    {
        assert(low_threshold <= midpoint);
        assert(midpoint <= high_threshold);
        if (val >= midpoint && val < high_threshold)
        {
            const std::string msg = "Thresholding " + std::to_string(val) + " to high threshold " + std::to_string(high_threshold) + "\n";
            std::cerr << msg << std::flush;
            return high_threshold;
        }
        else if (val < midpoint && val > low_threshold)
        {
            const std::string msg = "Thresholding " + std::to_string(val) + " to low threshold " + std::to_string(low_threshold) + "\n";
            std::cerr << msg << std::flush;
            return low_threshold;
        }
        return val;
    }

    inline constexpr float ColorChannelFromHex(uint8_t hexval)
    {
        return (float)hexval / 255.0f;
    }

    inline float TrimColorValue(const float val)
    {
        return ClampValue<float>(val, 0.0f, 1.0f);
    }

    inline uint8_t ColorChannelToHex(float colorval)
    {
        return (uint8_t)round(TrimColorValue(colorval) * 255.0f);
    }

    class RGBAColor
    {
    public:

        float r;
        float g;
        float b;
        float a;

        RGBAColor(const float r, const float g, const float b, const float a) : r(TrimColorValue(r)), g(TrimColorValue(g)), b(TrimColorValue(b)), a(TrimColorValue(a)) {}

        RGBAColor(const float r, const float g, const float b) : r(TrimColorValue(r)), g(TrimColorValue(g)), b(TrimColorValue(b)), a(1.0f) {}

        RGBAColor(const uint8_t r, const uint8_t g, const uint8_t b, const uint8_t a) : r(ColorChannelFromHex(r)), g(ColorChannelFromHex(g)), b(ColorChannelFromHex(b)), a(ColorChannelFromHex(a)) {}

        RGBAColor(const uint8_t r, const uint8_t g, const uint8_t b, const float a) : r(ColorChannelFromHex(r)), g(ColorChannelFromHex(g)), b(ColorChannelFromHex(b)), a(TrimColorValue(a)) {}

        RGBAColor() : r(0.0f), g(0.0f), b(0.0f), a(0.0f) {}

        inline float GetR() const
        {
            return r;
        }

        inline float GetG() const
        {
            return g;
        }

        inline float GetB() const
        {
            return b;
        }

        inline float GetA() const
        {
            return a;
        }

        inline void SetR(const float new_r)
        {
            r = TrimColorValue(new_r);
        }

        inline void SetG(const float new_g)
        {
            g = TrimColorValue(new_g);
        }

        inline void SetB(const float new_b)
        {
            b = TrimColorValue(new_b);
        }

        inline void SetA(const float new_a)
        {
            a = TrimColorValue(new_a);
        }

        inline uint8_t GetRHex() const
        {
            return ColorChannelToHex(r);
        }

        inline uint8_t GetGHex() const
        {
            return ColorChannelToHex(g);
        }

        inline uint8_t GetBHex() const
        {
            return ColorChannelToHex(b);
        }

        inline uint8_t GetAHex() const
        {
            return ColorChannelToHex(a);
        }

        inline void SetRHex(const uint8_t hex_r)
        {
            r = ColorChannelFromHex(hex_r);
        }

        inline void SetGHex(const uint8_t hex_g)
        {
            g = ColorChannelFromHex(hex_g);
        }

        inline void SetBHex(const uint8_t hex_b)
        {
            b = ColorChannelFromHex(hex_b);
        }

        inline void SetAHex(const uint8_t hex_a)
        {
            a = ColorChannelFromHex(hex_a);
        }
    };

    template<typename ColorType>
    class RGBAColorBuilder
    {
    private:

        RGBAColorBuilder() {}

    public:

        static inline ColorType MakeFromFloatColors(const float r, const float g, const float b, const float a=1.0f)
        {
            ColorType color;
            color.r = TrimColorValue(r);
            color.g = TrimColorValue(g);
            color.b = TrimColorValue(b);
            color.a = TrimColorValue(a);
            return color;
        }

        static inline ColorType MakeFromHexColors(const uint8_t r, const uint8_t g, const uint8_t b, const uint8_t a=0xff)
        {
            return MakeFromFloatColors(ColorChannelFromHex(r), ColorChannelFromHex(g), ColorChannelFromHex(b), ColorChannelFromHex(a));
        }

        static inline ColorType MakeFromMixedColors(const uint8_t r, const uint8_t g, const uint8_t b, const float a=1.0f)
        {
            return MakeFromFloatColors(ColorChannelFromHex(r), ColorChannelFromHex(g), ColorChannelFromHex(b), TrimColorValue(a));
        }

        static inline ColorType InterpolateHotToCold(const double value, const double min_value=0.0, const double max_value=1.0)
        {
            assert(min_value < max_value);
            const double real_value = ClampValue(value, min_value, max_value);
            const double range = max_value - min_value;
            // Start with white
            double r = 1.0;
            double g = 1.0;
            double b = 1.0;
            // Interpolate
            if (real_value < (min_value + (0.25 * range)))
            {
                r = 0.0;
                g = 4.0 * (real_value - min_value) / range;
            }
            else if (real_value < (min_value + (0.5 * range)))
            {
                r = 0.0;
                b = 1.0 + 4.0 * (min_value + 0.25 * range - real_value) / range;
            }
            else if (real_value < (min_value + (0.75 * range)))
            {
                r = 4.0 * (real_value - min_value - 0.5 * range) / range;
                b = 0.0;
            }
            else
            {
                g = 1.0 + 4.0 * (min_value + 0.75 * range - real_value) / range;
                b = 0.0;
            }
            return MakeFromFloatColors((float)r, (float)g, (float)b, 1.0f);
        }

    };

    template<typename ColorTypeA, typename ColorTypeB>
    inline ColorTypeB ConvertColor(const ColorTypeA& color)
    {
        ColorTypeB cvt_color;
        cvt_color.r = TrimColorValue(color.r);
        cvt_color.g = TrimColorValue(color.g);
        cvt_color.b = TrimColorValue(color.b);
        cvt_color.a = TrimColorValue(color.a);
        return cvt_color;
    }

    template<typename ColorType>
    inline ColorType GenerateUniqueColor(const uint32_t color_code, const float alpha=1.0f)
    {
        // For color_code < 22, we pick from a table
        if (color_code == 0)
        {
            return RGBAColorBuilder<ColorType>::MakeFromFloatColors(1.0f, 1.0f, 1.0f, 0.0f);
        }
        else if (color_code <= 20)
        {
            // CHECK TO MAKE SURE RGB/RBG IS CORRECT!
            if (color_code == 1)
            {
                return RGBAColorBuilder<ColorType>::MakeFromMixedColors(0xff, 0x00, 0xb3, alpha);
            }
            else if (color_code == 2)
            {
                return RGBAColorBuilder<ColorType>::MakeFromMixedColors(0x80, 0x75, 0x3e, alpha);
            }
            else if (color_code == 3)
            {
                return RGBAColorBuilder<ColorType>::MakeFromMixedColors(0xff, 0x00, 0x68, alpha);
            }
            else if (color_code == 4)
            {
                return RGBAColorBuilder<ColorType>::MakeFromMixedColors(0xa6, 0xd7, 0xbd, alpha);
            }
            else if (color_code == 5)
            {
                return RGBAColorBuilder<ColorType>::MakeFromMixedColors(0xc1, 0x20, 0x00, alpha);
            }
            else if (color_code == 6)
            {
                return RGBAColorBuilder<ColorType>::MakeFromMixedColors(0xce, 0x62, 0xa2, alpha);
            }
            else if (color_code == 7)
            {
                return RGBAColorBuilder<ColorType>::MakeFromMixedColors(0x81, 0x66, 0x70, alpha);
            }
            else if (color_code == 8)
            {
                return RGBAColorBuilder<ColorType>::MakeFromMixedColors(0x00, 0x34, 0x7d, alpha);
            }
            else if (color_code == 9)
            {
                return RGBAColorBuilder<ColorType>::MakeFromMixedColors(0xf6, 0x8e, 0x76, alpha);
            }
            else if (color_code == 10)
            {
                return RGBAColorBuilder<ColorType>::MakeFromMixedColors(0x00, 0x8a, 0x53, alpha);
            }
            else if (color_code == 11)
            {
                return RGBAColorBuilder<ColorType>::MakeFromMixedColors(0xff, 0x5c, 0x7a, alpha);
            }
            else if (color_code == 12)
            {
                return RGBAColorBuilder<ColorType>::MakeFromMixedColors(0x53, 0x7a, 0x37, alpha);
            }
            else if (color_code == 13)
            {
                return RGBAColorBuilder<ColorType>::MakeFromMixedColors(0xff, 0x00, 0x8e, alpha);
            }
            else if (color_code == 14)
            {
                return RGBAColorBuilder<ColorType>::MakeFromMixedColors(0xb3, 0x51, 0x28, alpha);
            }
            else if (color_code == 15)
            {
                return RGBAColorBuilder<ColorType>::MakeFromMixedColors(0xf4, 0x00, 0xc8, alpha);
            }
            else if (color_code == 16)
            {
                return RGBAColorBuilder<ColorType>::MakeFromMixedColors(0x7f, 0x0d, 0x18, alpha);
            }
            else if (color_code == 17)
            {
                return RGBAColorBuilder<ColorType>::MakeFromMixedColors(0x93, 0x00, 0xaa, alpha);
            }
            else if (color_code == 18)
            {
                return RGBAColorBuilder<ColorType>::MakeFromMixedColors(0x59, 0x15, 0x33, alpha);
            }
            else if (color_code == 19)
            {
                return RGBAColorBuilder<ColorType>::MakeFromMixedColors(0xf1, 0x13, 0x3a, alpha);
            }
            else
            {
                return RGBAColorBuilder<ColorType>::MakeFromMixedColors(0x23, 0x16, 0x2c, alpha);
            }
        }
        else
        {
            return RGBAColorBuilder<ColorType>::MakeFromFloatColors(0.0f, 0.0f, 0.0f, alpha);
        }
    }

    inline size_t GetNumOMPThreads()
    {
        #if defined(_OPENMP)
        size_t num_threads = 0;
        #pragma omp parallel
        {
            num_threads = (size_t)omp_get_num_threads();
        }
        return num_threads;
        #else
        return 1;
        #endif
    }

    template<typename Datatype, typename Allocator=std::allocator<Datatype>>
    inline Eigen::MatrixXd BuildDistanceMatrix(const std::vector<Datatype, Allocator>& data, const std::function<double(const Datatype&, const Datatype&)>& distance_fn)
    {
        Eigen::MatrixXd distance_matrix(data.size(), data.size());
#ifdef ENABLE_PARALLEL_DISTANCE_MATRIX
        #pragma omp parallel for
#endif
        for (size_t idx = 0; idx < data.size(); idx++)
        {
            for (size_t jdx = idx; jdx < data.size(); jdx++)
            {
                if (idx != jdx)
                {
                    const double distance = distance_fn(data[idx], data[jdx]);
                    distance_matrix((ssize_t)idx, (ssize_t)jdx) = distance;
                    distance_matrix((ssize_t)jdx, (ssize_t)idx) = distance;
                }
                else
                {
                    distance_matrix((ssize_t)idx, (ssize_t)jdx) = 0.0;
                    distance_matrix((ssize_t)jdx, (ssize_t)idx) = 0.0;
                }
            }
        }
        return distance_matrix;
    }

    template<typename FirstDatatype, typename SecondDatatype, typename FirstAllocator=std::allocator<FirstDatatype>, typename SecondAllocator=std::allocator<SecondDatatype>>
    inline Eigen::MatrixXd BuildDistanceMatrix(const std::vector<FirstDatatype, FirstAllocator>& data1, const std::vector<SecondDatatype, SecondAllocator>& data2, const std::function<double(const FirstDatatype&, const SecondDatatype&)>& distance_fn)
    {
        Eigen::MatrixXd distance_matrix(data1.size(), data1.size());
#ifdef ENABLE_PARALLEL_DISTANCE_MATRIX
        #pragma omp parallel for
#endif
        for (size_t idx = 0; idx < data1.size(); idx++)
        {
            for (size_t jdx = 0; jdx < data2.size(); jdx++)
            {
                const double distance = distance_fn(data1[idx], data2[jdx]);
                distance_matrix((ssize_t)idx, (ssize_t)jdx) = distance;
                distance_matrix((ssize_t)jdx, (ssize_t)idx) = distance;
            }
        }
        return distance_matrix;
    }

    template<typename Item, typename Value, typename ItemAlloc=std::allocator<Item>>
    std::vector<std::pair<int64_t, double>> GetKNearestNeighbors(const std::vector<Item, ItemAlloc>& items, const Value& current, const std::function<double(const Item&, const Value&)>& distance_fn, const size_t K)
    {
        if (K == 0)
        {
            return std::vector<std::pair<int64_t, double>>();
        }
        if (items.size() > K)
        {
            std::function<bool(const std::pair<int64_t, double>&, const std::pair<int64_t, double>&)> compare_fn = [] (const std::pair<int64_t, double>& index1, const std::pair<int64_t, double>& index2) { return index1.second < index2.second; };
            std::vector<std::vector<std::pair<int64_t, double>>> per_thread_nearests(GetNumOMPThreads(), std::vector<std::pair<int64_t, double>>(K, std::make_pair(-1, std::numeric_limits<double>::infinity())));
    #ifdef ENABLE_PARALLEL_K_NEAREST_NEIGHBORS
            #pragma omp parallel for
    #endif
            for (size_t idx = 0; idx < items.size(); idx++)
            {
                const Item& item = items[idx];
                const double distance = distance_fn(item, current);
#ifdef ENABLE_PARALLEL_K_NEAREST_NEIGHBORS
                #if defined(_OPENMP)
                const size_t thread_num = (size_t)omp_get_thread_num();
                #else
                const size_t thread_num = 0;
                #endif
#else
                const size_t thread_num = 0;
#endif
                std::vector<std::pair<int64_t, double>>& current_thread_nearests = per_thread_nearests[thread_num];
                auto itr = std::max_element(current_thread_nearests.begin(), current_thread_nearests.end(), compare_fn);
                const double worst_distance = itr->second;
                if (worst_distance > distance)
                {
                    itr->first = (int64_t)idx;
                    itr->second = distance;
                }
            }
            std::vector<std::pair<int64_t, double>> k_nearests;
            k_nearests.reserve(K);
            for (size_t thread_idx = 0; thread_idx < per_thread_nearests.size(); thread_idx++)
            {
                const std::vector<std::pair<int64_t, double>>& thread_nearests = per_thread_nearests[thread_idx];
                for (size_t nearest_idx = 0; nearest_idx < thread_nearests.size(); nearest_idx++)
                {
                    const std::pair<int64_t, double> current_ith_nearest = thread_nearests[nearest_idx];
                    if (!std::isinf(current_ith_nearest.second) && current_ith_nearest.first != -1)
                    {
                        if (k_nearests.size() < K)
                        {
                            k_nearests.push_back(current_ith_nearest);
                        }
                        else
                        {
                            auto itr = std::max_element(k_nearests.begin(), k_nearests.end(), compare_fn);
                            const double worst_distance = itr->second;
                            if (worst_distance > current_ith_nearest.second)
                            {
                                itr->first = current_ith_nearest.first;
                                itr->second = current_ith_nearest.second;
                            }
                        }
                    }
                }
            }
            k_nearests.shrink_to_fit();
            return k_nearests;
        }
        else
        {
            std::vector<std::pair<int64_t, double>> k_nearests(items.size(), std::make_pair(-1, std::numeric_limits<double>::infinity()));
#ifdef ENABLE_PARALLEL_K_NEAREST_NEIGHBORS
            #pragma omp parallel for
#endif
            for (size_t idx = 0; idx < items.size(); idx++)
            {
                const Item& item = items[idx];
                const double distance = distance_fn(item, current);
                k_nearests[idx] = std::make_pair((int64_t)idx, distance);
            }
            return k_nearests;
        }
    }

    class SplitMix64PRNG
    {
    private:

        uint64_t state_; /* The state can be seeded with any value. */

        inline uint64_t next(void)
        {
            uint64_t z = (state_ += UINT64_C(0x9E3779B97F4A7C15));
            z = (z ^ (z >> 30)) * UINT64_C(0xBF58476D1CE4E5B9);
            z = (z ^ (z >> 27)) * UINT64_C(0x94D049BB133111EB);
            return z ^ (z >> 31);
        }

    public:

        inline SplitMix64PRNG(const uint64_t seed_val)
        {
            seed(seed_val);
        }

        static constexpr uint64_t min(void)
        {
            return 0u;
        }

        static constexpr uint64_t max(void)
        {
            return std::numeric_limits<uint64_t>::max();
        }

        inline void seed(const uint64_t seed_val)
        {
            state_ = seed_val;
        }

        inline void discard(const unsigned long long z)
        {
            uint64_t temp __attribute__((unused)); // This suppresses "set but not used" warnings
            temp = 0u;
            for (unsigned long long i = 0; i < z; i++)
            {
                temp = next();
                __asm__ __volatile__(""); // This should prevent the compiler from optimizing out the loop
            }
        }

        inline uint64_t operator() (void)
        {
            return next();
        }
    };

    class XorShift128PlusPRNG
    {
    private:

        uint64_t state_1_;
        uint64_t state_2_;

        inline uint64_t next(void)
        {
            uint64_t s1 = state_1_;
            const uint64_t s0 = state_2_;
            state_1_ = s0;
            s1 ^= s1 << 23; // a
            state_2_ = s1 ^ s0 ^ (s1 >> 18) ^ (s0 >> 5); // b, c
            return state_2_ + s0;
        }

    public:

        inline XorShift128PlusPRNG(const uint64_t seed_val)
        {
            seed(seed_val);
        }

        static constexpr uint64_t min(void)
        {
            return 0u;
        }

        static constexpr uint64_t max(void)
        {
            return std::numeric_limits<uint64_t>::max();
        }

        inline void seed(const uint64_t seed_val)
        {
            SplitMix64PRNG temp_seed_gen(seed_val);
            state_1_ = temp_seed_gen();
            state_2_ = temp_seed_gen();
        }

        inline void discard(const unsigned long long z)
        {
            uint64_t temp __attribute__((unused)); // This suppresses "set but not used" warnings
            temp = 0u;
            for (unsigned long long i = 0; i < z; i++)
            {
                temp = next();
                __asm__ __volatile__(""); // This should prevent the compiler from optimizing out the loop
            }
        }

        inline uint64_t operator() (void)
        {
            return next();
        }
    };

    class XorShift1024StarPRNG
    {
    private:

        std::array<uint64_t, 16> state_;
        int32_t p;

        inline uint64_t next(void)
        {
            const uint64_t s0 = state_[(size_t)p];
            p = (p + 1) & 15;
            uint64_t s1 = state_[(size_t)p];
            s1 ^= s1 << 31; // a
            state_[(size_t)p] = s1 ^ s0 ^ (s1 >> 11) ^ (s0 >> 30); // b,c
            return state_[(size_t)p] * UINT64_C(1181783497276652981);
        }

    public:

        inline XorShift1024StarPRNG(const uint64_t seed_val)
        {
            seed(seed_val);
            p = 0;
        }

        static constexpr uint64_t min(void)
        {
            return 0u;
        }

        static constexpr uint64_t max(void)
        {
            return std::numeric_limits<uint64_t>::max();
        }

        inline void seed(const uint64_t seed_val)
        {
            SplitMix64PRNG temp_seed_gen(seed_val);
            for (size_t idx = 0u; idx < state_.size(); idx++)
            {
                state_[idx] = temp_seed_gen();
            }
        }

        inline void discard(const unsigned long long z)
        {
            uint64_t temp __attribute__((unused)); // This suppresses "set but not used" warnings
            temp = 0u;
            for (unsigned long long i = 0; i < z; i++)
            {
                temp = next();
                __asm__ __volatile__(""); // This should prevent the compiler from optimizing out the loop
            }
        }

        inline uint64_t operator() (void)
        {
            return next();
        }
    };


    // SEE https://people.sc.fsu.edu/~jburkardt/presentations/truncated_normal.pdf FOR DETAILS
    inline double EvaluateGaussianCDF(const double mean, const double std_dev, const double val)
    {
        return 0.5 * (1.0 + std::erf( ( (val - mean) / std_dev ) / sqrt(2.0) ) );
    }

    inline double EvaluateGaussianPDF(const double mean, const double std_dev, const double val)
    {
        const double exponent = ((val - mean) * (val - mean)) / (2.0 * std_dev * std_dev);
        const double fraction = 1.0 / (std_dev * std::sqrt(2.0 * M_PI));
        const double pdf = fraction * std::exp(-exponent);
        return pdf;
    }

    inline double EvaluateTruncatedGaussianCDF(const double mean, const double lower_bound, const double upper_bound, const double std_dev, const double val)
    {
        assert(lower_bound <= upper_bound);
        if (val <= lower_bound)
        {
            return 0.0;
        }
        else if (val >= upper_bound)
        {
            return 1.0;
        }
        else
        {
            const double cdf_lower_bound = EvaluateGaussianCDF(mean, std_dev, lower_bound);
            const double numerator = EvaluateGaussianCDF(mean, std_dev, val) - cdf_lower_bound;
            const double denominator = EvaluateGaussianCDF(mean, std_dev, upper_bound) - cdf_lower_bound;
            return numerator / denominator;
        }
    }

    inline double EvaluateTruncatedGaussianPDF(const double mean, const double lower_bound, const double upper_bound, const double std_dev, const double val)
    {
        assert(lower_bound <= upper_bound);
        if (val <= lower_bound)
        {
            return 0.0;
        }
        else if (val >= upper_bound)
        {
            return 0.0;
        }
        else
        {
            const double cdf_upper = EvaluateGaussianCDF(mean, std_dev, upper_bound);
            const double cdf_lower = EvaluateGaussianCDF(mean, std_dev, lower_bound);
            const double probability_enclosed = cdf_upper - cdf_lower;
            const double gaussian_pdf = EvaluateGaussianPDF(mean, std_dev, val);
            const double pdf = gaussian_pdf / probability_enclosed;
            return pdf;
        }
    }

    inline double IntegrateGaussian(const double mean, const double std_dev, const double lower_limit, const double upper_limit)
    {
        assert(lower_limit <= upper_limit);
        const double upper_limit_cdf = EvaluateGaussianCDF(mean, std_dev, upper_limit);
        const double lower_limit_cdf = EvaluateGaussianCDF(mean, std_dev, lower_limit);
        const double probability = upper_limit_cdf - lower_limit_cdf;
        //const std::string msg = "Integrated Gaussian with mean " + std::to_string(mean) + " std.dev. " + std::to_string(std_dev) + " over range [" + std::to_string(lower_limit) + "," + std::to_string(upper_limit) + "] to be " + std::to_string(probability);
        //std::cout << msg << std::endl;
        return probability;
    }

    inline double IntegrateTruncatedGaussian(const double mean, const double lower_bound, const double upper_bound, const double std_dev, const double lower_limit, const double upper_limit)
    {
        assert(lower_bound <= upper_bound);
        assert(lower_limit <= upper_limit);
        const double lower_limit_cdf = EvaluateTruncatedGaussianCDF(mean, lower_bound, upper_bound, std_dev, lower_limit);
        const double upper_limit_cdf = EvaluateTruncatedGaussianCDF(mean, lower_bound, upper_bound, std_dev, upper_limit);
        const double probability = upper_limit_cdf - lower_limit_cdf;
        //const std::string msg = "Integrated truncated Gaussian with mean " + std::to_string(mean) + " std.dev. " + std::to_string(std_dev) + " and bounds [" + std::to_string(lower_bound) + "," + std::to_string(upper_bound) + "] over range [" + std::to_string(lower_limit) + "," + std::to_string(upper_limit) + "] to be " + std::to_string(probability);
        //std::cout << msg << std::endl;
        return probability;
    }

    class TruncatedNormalDistribution
    {
    protected:

        double mean_;
        double stddev_;
        double std_lower_bound_;
        double std_upper_bound_;

        enum CASES {TYPE_1, TYPE_2, TYPE_3, TYPE_4, NONE};
        CASES case_;
        std::uniform_real_distribution<double> uniform_unit_dist_;
        std::uniform_real_distribution<double> uniform_range_dist_;
        std::exponential_distribution<double> exponential_dist_;
        std::normal_distribution<double> normal_dist_;

        inline bool CheckSimple(const double lower_bound, const double upper_bound) const
        {
            // Init Values Used in Inequality of Interest
            const double val1 = (2 * sqrt(exp(1))) / (lower_bound + sqrt(pow(lower_bound, 2) + 4));
            const double val2 = exp((pow(lower_bound, 2) - lower_bound * sqrt(pow(lower_bound, 2) + 4)) / (4));
            if (upper_bound > lower_bound + val1 * val2)
            {
                return true;
            }
            else
            {
                return false;
            }
        }

        // Naive Accept-Reject algorithm
        template<typename Generator>
        inline double NaiveAcceptReject(const double lower_bound, const double upper_bound, Generator& prng)
        {
            while (true)
            {
                const double draw = normal_dist_(prng); // Rf_rnorm(0.0, 1.0) ; // Normal distribution (i.e. std::normal_distribution<double>)
                if ((draw <= upper_bound) && (draw >= lower_bound))
                {
                    return draw;
                }
            }
        }

        // Accept-Reject Algorithm
        template<typename Generator>
        inline double SimpleAcceptReject(const double lower_bound, Generator& prng)
        {
            // Init Values
            const double alpha = (lower_bound + sqrt(pow(lower_bound, 2) + 4.0)) / (2.0) ;
            while (true)
            {
                const double e = exponential_dist_(prng); // Rf_rexp(1.0) ; // Exponential distribution (i.e. std::exponential_distribution<double>)
                const double z = lower_bound + e / alpha;
                const double rho = exp(-pow(alpha - z, 2) / 2);
                const double u = uniform_unit_dist_(prng); //  Rf_runif(0, 1) ; // Uniform distribution (i.e. std::uniform_real_distribution<double>)
                if (u <= rho)
                {
                    return z;
                }
            }
        }

        // Accept-Reject Algorithm
        template<typename Generator>
        inline double ComplexAcceptReject(const double lower_bound, const double upper_bound, Generator& prng)
        {
            while (true)
            {
                const double z = uniform_range_dist_(prng); // Rf_runif(lower_bound, upper_bound) ; // Uniform distribution (i.e. std::uniform_real_distribution<double>)
                double rho = 0.0;
                if (0 < lower_bound)
                {
                    rho = exp((pow(lower_bound, 2) - pow(z, 2)) / 2);
                }
                else if (upper_bound < 0)
                {
                    rho = exp((pow(upper_bound, 2) - pow(z, 2)) / 2);
                }
                else if (0 < upper_bound && lower_bound < 0)
                {
                    rho = exp(- pow(z, 2) / 2);
                }
                const double u = uniform_unit_dist_(prng); // Rf_runif(0, 1) ; // Uniform distribution (i.e. std::uniform_real_distribution<double>)
                if (u <= rho)
                {
                    return z;
                }
            }
        }

        template<typename Generator>
        inline double Sample(Generator& prng)
        {
            if (case_ == TYPE_1)
            {
                const double draw = NaiveAcceptReject(std_lower_bound_, std_upper_bound_, prng);
                return mean_ + stddev_ * draw;
            }
            else if (case_ == TYPE_2)
            {
                const double draw = SimpleAcceptReject(std_lower_bound_, prng);
                return mean_ + stddev_ * draw;
            }
            else if (case_ == TYPE_3)
            {
                while (true)
                {
                    const double draw = SimpleAcceptReject(std_lower_bound_, prng); // use the simple algorithm if it is more efficient
                    if (draw <= std_upper_bound_)
                    {
                        return mean_ + stddev_ * draw;
                    }
                }
            }
            else if (case_ == TYPE_4)
            {
                const double draw = ComplexAcceptReject(std_lower_bound_, std_upper_bound_, prng);
                return mean_ + stddev_ * draw;
            }
            else
            {
                assert(case_ == NONE);
                return mean_;
            }
        }

    public:

        inline TruncatedNormalDistribution(const double mean, const double stddev, const double lower_bound, const double upper_bound) : uniform_unit_dist_(0.0, 1.0), uniform_range_dist_(lower_bound, upper_bound), exponential_dist_(1.0), normal_dist_(0.0, 1.0)
        {
            // Set operating parameters
            mean_ = mean;
            stddev_ = stddev;
            if (fabs(stddev_) == 0.0)
            {
                case_ = NONE;
            }
            else
            {
                // Standardize the lower and upper bounds
                std_lower_bound_ = (lower_bound - mean_) / stddev_;
                std_upper_bound_ = (upper_bound - mean_) / stddev_;
                // Set the operating case - i.e. which sampling method we will use
                case_ = NONE;
                if (0.0 <= std_upper_bound_ && 0.0 >= std_lower_bound_)
                {
                    case_ = TYPE_1;
                }
                if (0.0 < std_lower_bound_ && std_upper_bound_ == INFINITY)
                {
                    case_ = TYPE_2;
                }
                if (0.0 > std_upper_bound_ && std_lower_bound_ == -INFINITY)
                {
                    std_lower_bound_ = -1 * std_upper_bound_;
                    std_upper_bound_ = INFINITY;
                    stddev_ = -1 * stddev_;
                    case_ = TYPE_2;
                }
                if ((0.0 > std_upper_bound_ || 0.0 < std_lower_bound_) && !(std_upper_bound_ == INFINITY || std_lower_bound_ == -INFINITY))
                {
                    if (CheckSimple(std_lower_bound_, std_upper_bound_))
                    {
                        case_ = TYPE_3;
                    }
                    else
                    {
                        case_ = TYPE_4;
                    }
                }
                assert((case_ == TYPE_1) || (case_ == TYPE_2) || (case_ == TYPE_3) || (case_ == TYPE_4));
            }
        }

        template<typename Generator>
        inline double operator()(Generator& prng)
        {
            return Sample(prng);
        }
    };

    class MultivariteGaussianDistribution
    {
    protected:
        const Eigen::VectorXd mean_;
        const Eigen::MatrixXd norm_transform_;

        std::normal_distribution<double> unit_gaussian_dist_;

        template<typename Generator>
        inline Eigen::VectorXd Sample(Generator& prng)
        {
            Eigen::VectorXd draw;
            draw.resize(mean_.rows());

            for (ssize_t idx = 0; idx < draw.rows(); idx++)
            {
                draw(idx) = unit_gaussian_dist_(prng);
            }

            return norm_transform_ * draw + mean_;
        }

        static Eigen::MatrixXd CalculateNormTransform(const Eigen::MatrixXd& covariance)
        {
            Eigen::MatrixXd norm_transform;

            Eigen::LLT<Eigen::MatrixXd> chol_solver(covariance);

            if (chol_solver.info() == Eigen::Success)
            {
                // Use cholesky solver
                norm_transform = chol_solver.matrixL();
            }
            else
            {
                // Use eigen solver
                Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigen_solver(covariance);
                norm_transform = eigen_solver.eigenvectors() * eigen_solver.eigenvalues().cwiseMax(0.0).cwiseSqrt().asDiagonal();
            }

            return norm_transform;
        }

    public:
        inline MultivariteGaussianDistribution(const Eigen::VectorXd& mean, const Eigen::MatrixXd& covariance) : mean_(mean), norm_transform_(CalculateNormTransform(covariance)), unit_gaussian_dist_(0.0, 1.0)
        {
            assert(mean.rows() == covariance.rows());
            assert(covariance.cols() == covariance.rows());

            assert(!(norm_transform_.unaryExpr([] (const double &val) { return std::isnan(val); })).any() && "NaN Found in norm_transform in MultivariateGaussianDistribution");
            assert(!(norm_transform_.unaryExpr([] (const double &val) { return std::isinf(val); })).any() && "Inf Found in norm_transform in MultivariateGaussianDistribution");
        }

        template<typename Generator>
        inline Eigen::VectorXd operator()(Generator& prng)
        {
            return Sample(prng);
        }
    };

    class RandomRotationGenerator
    {
    protected:

        std::uniform_real_distribution<double> uniform_unit_dist_;

    public:

        inline RandomRotationGenerator() : uniform_unit_dist_(0.0, 1.0) {}

        // From: "Uniform Random Rotations", Ken Shoemake, Graphics Gems III, pg. 124-132
        static inline Eigen::Quaterniond GenerateUniformRandomQuaternion(const std::function<double()>& uniform_unit_dist)
        {
            const double x0 = uniform_unit_dist();
            const double r1 = sqrt(1.0 - x0);
            const double r2 = sqrt(x0);
            const double t1 = 2.0 * M_PI * uniform_unit_dist();
            const double t2 = 2.0 * M_PI * uniform_unit_dist();
            const double c1 = cos(t1);
            const double s1 = sin(t1);
            const double c2 = cos(t2);
            const double s2 = sin(t2);
            const double x = s1 * r1;
            const double y = c1 * r1;
            const double z = s2 * r2;
            const double w = c2 * r2;
            return Eigen::Quaterniond(w, x, y, z);
        }

        // From Effective Sampling and Distance Metrics for 3D Rigid Body Path Planning, by James Kuffner, ICRA 2004
        static inline Eigen::Vector3d GenerateUniformRandomEulerAngles(const std::function<double()>& uniform_unit_dist)
        {
            const double roll = 2.0 * M_PI * uniform_unit_dist() -  M_PI;
            const double pitch_init = std::acos(1.0 - (2.0 * uniform_unit_dist())) + M_PI_2;
            const double pitch = (uniform_unit_dist() < 0.5) ? ((pitch_init < M_PI) ? pitch_init + M_PI : pitch_init - M_PI) : pitch_init;
            const double yaw = 2.0 * M_PI * uniform_unit_dist() -  M_PI;
            return Eigen::Vector3d(roll, pitch, yaw);
        }

        template<typename Generator>
        inline Eigen::Quaterniond GetQuaternion(Generator& prng)
        {
            std::function<double()> uniform_rand_fn = [&] () { return uniform_unit_dist_(prng); };
            return GenerateUniformRandomQuaternion(uniform_rand_fn);
        }

        template<typename Generator>
        inline std::vector<double> GetRawQuaternion(Generator& prng)
        {
            const Eigen::Quaterniond quat = GetQuaternion(prng);
            return std::vector<double>{quat.x(), quat.y(), quat.z(), quat.w()};
        }

        template<typename Generator>
        inline Eigen::Vector3d GetEulerAngles(Generator& prng)
        {
            std::function<double()> uniform_rand_fn = [&] () { return uniform_unit_dist_(prng); };
            return GenerateUniformRandomEulerAngles(uniform_rand_fn);
        }

        template<typename Generator>
        inline std::vector<double> GetRawEulerAngles(Generator& prng)
        {
            const Eigen::Vector3d angles = GetEulerAngles(prng);
            return std::vector<double>{angles.x(), angles.y(), angles.z()};
        }
    };

    /////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////                                       PROTOTYPES ONLY                                         /////
    ///// Specializations for specific types - if you want a specialization for a new type, add it here /////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////

    template<typename T>
    inline uint64_t SerializeFixedSizePOD(const T& item_to_serialize, std::vector<uint8_t>& buffer);

    template<typename T>
    inline std::pair<T, uint64_t> DeserializeFixedSizePOD(const std::vector<uint8_t>& buffer, const uint64_t current);

    template<typename T, typename Allocator=std::allocator<T>>
    inline uint64_t SerializeVector(const std::vector<T, Allocator>& vec_to_serialize, std::vector<uint8_t>& buffer, const std::function<uint64_t(const T&, std::vector<uint8_t>&)>& item_serializer);

    template<typename T, typename Allocator=std::allocator<T>>
    inline std::pair<std::vector<T, Allocator>, uint64_t> DeserializeVector(const std::vector<uint8_t>& buffer, const uint64_t current, const std::function<std::pair<T, uint64_t>(const std::vector<uint8_t>&, const uint64_t)>& item_deserializer);

    template<typename Key, typename T, typename Compare = std::less<Key>, typename Allocator = std::allocator<std::pair<const Key, T>>>
    inline uint64_t SerializeMap(const std::map<Key, T, Compare, Allocator>& map_to_serialize, std::vector<uint8_t>& buffer, const std::function<uint64_t(const Key&, std::vector<uint8_t>&)>& key_serializer, const std::function<uint64_t(const T&, std::vector<uint8_t>&)>& value_serializer);

    template<typename Key, typename T, typename Compare = std::less<Key>, typename Allocator = std::allocator<std::pair<const Key, T>>>
    inline std::pair<std::map<Key, T, Compare, Allocator>, uint64_t> DeserializeMap(const std::vector<uint8_t>& buffer, const uint64_t current, const std::function<std::pair<Key, uint64_t>(const std::vector<uint8_t>&, const uint64_t)>& key_deserializer, const std::function<std::pair<T, uint64_t>(const std::vector<uint8_t>&, const uint64_t)>& value_deserializer);

    template<typename First, typename Second>
    inline uint64_t SerializePair(const std::pair<First, Second>& pair_to_serialize, std::vector<uint8_t>& buffer, const std::function<uint64_t(const First&, std::vector<uint8_t>&)>& first_serializer, const std::function<uint64_t(const Second&, std::vector<uint8_t>&)>& second_serializer);

    template<typename First, typename Second>
    inline const std::pair<std::pair<First, Second>, uint64_t> DeserializePair(const std::vector<uint8_t>& buffer, const uint64_t current, const std::function<std::pair<First, uint64_t>(const std::vector<uint8_t>&, const uint64_t)>& first_deserializer, const std::function<std::pair<Second, uint64_t>(const std::vector<uint8_t>&, const uint64_t)>& second_deserializer);

    /////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////                                   IMPLEMENTATIONS ONLY                                        /////
    ///// Specializations for specific types - if you want a specialization for a new type, add it here /////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////

    template<typename T>
    inline uint64_t SerializeFixedSizePOD(const T& item_to_serialize, std::vector<uint8_t>& buffer)
    {
        const uint64_t start_buffer_size = buffer.size();
        // Fixed-size serialization via memcpy
        std::vector<uint8_t> temp_buffer(sizeof(item_to_serialize), 0x00);
        memcpy(&temp_buffer[0], &item_to_serialize, sizeof(item_to_serialize));
        // Move to buffer
        buffer.insert(buffer.end(), temp_buffer.begin(), temp_buffer.end());
        // Figure out how many bytes were written
        const uint64_t end_buffer_size = buffer.size();
        const uint64_t bytes_written = end_buffer_size - start_buffer_size;
        return bytes_written;
    }

    template<typename T>
    inline std::pair<T, uint64_t> DeserializeFixedSizePOD(const std::vector<uint8_t>& buffer, const uint64_t current)
    {
        T temp_item;
        assert(current <= buffer.size());
        assert((current + sizeof(temp_item)) <= buffer.size());
        memcpy(&temp_item, &buffer[current], sizeof(temp_item));
        return std::make_pair(temp_item, sizeof(temp_item));
    }

    template<typename CharType>
    inline uint64_t SerializeString(const std::basic_string<CharType>& str_to_serialize, std::vector<uint8_t>& buffer)
    {
        const uint64_t start_buffer_size = buffer.size();
        // First, write a uint64_t size header
        const uint64_t size = (uint64_t)str_to_serialize.size();
        SerializeFixedSizePOD<uint64_t>(size, buffer);
        // Serialize the contained items
        for (size_t idx = 0; idx < size; idx++)
        {
            const CharType& current = str_to_serialize[idx];
            SerializeFixedSizePOD<CharType>(current, buffer);
        }
        // Figure out how many bytes were written
        const uint64_t end_buffer_size = buffer.size();
        const uint64_t bytes_written = end_buffer_size - start_buffer_size;
        return bytes_written;
    }

    template<typename CharType>
    inline std::pair<std::basic_string<CharType>, uint64_t> DeserializeString(const std::vector<uint8_t>& buffer, const uint64_t current)
    {
        // First, try to load the header
        assert(current < buffer.size());
        uint64_t current_position = current;
        // Load the header
        const std::pair<uint64_t, uint64_t> deserialized_size = DeserializeFixedSizePOD<uint64_t>(buffer, current_position);
        const uint64_t size = deserialized_size.first;
        current_position += deserialized_size.second;
        // Deserialize the items
        std::basic_string<CharType> deserialized;
        deserialized.reserve(size);
        for (uint64_t idx = 0; idx < size; idx++)
        {
            const std::pair<CharType, uint64_t> deserialized_char = DeserializeFixedSizePOD<CharType>(buffer, current_position);
            deserialized.push_back(deserialized_char.first);
            current_position += deserialized_char.second;
        }
        deserialized.shrink_to_fit();
        // Figure out how many bytes were read
        const uint64_t bytes_read = current_position - current;
        return std::make_pair(deserialized, bytes_read);
    }

    template<typename T, typename Allocator>
    inline uint64_t SerializeVector(const std::vector<T, Allocator>& vec_to_serialize, std::vector<uint8_t>& buffer, const std::function<uint64_t(const T&, std::vector<uint8_t>&)>& item_serializer)
    {
        const uint64_t start_buffer_size = buffer.size();
        // First, write a uint64_t size header
        const uint64_t size = (uint64_t)vec_to_serialize.size();
        SerializeFixedSizePOD<uint64_t>(size, buffer);
        // Serialize the contained items
        for (size_t idx = 0; idx < size; idx++)
        {
            const T& current = vec_to_serialize[idx];
            item_serializer(current, buffer);
        }
        // Figure out how many bytes were written
        const uint64_t end_buffer_size = buffer.size();
        const uint64_t bytes_written = end_buffer_size - start_buffer_size;
        return bytes_written;
    }

    template<typename T, typename Allocator>
    inline std::pair<std::vector<T, Allocator>, uint64_t> DeserializeVector(const std::vector<uint8_t>& buffer, const uint64_t current, const std::function<std::pair<T, uint64_t>(const std::vector<uint8_t>&, const uint64_t)>& item_deserializer)
    {
        // First, try to load the header
        assert(current < buffer.size());
        uint64_t current_position = current;
        // Load the header
        const std::pair<uint64_t, uint64_t> deserialized_size = DeserializeFixedSizePOD<uint64_t>(buffer, current_position);
        const uint64_t size = deserialized_size.first;
        current_position += deserialized_size.second;
        // Deserialize the items
        std::vector<T, Allocator> deserialized;
        deserialized.reserve(size);
        for (uint64_t idx = 0; idx < size; idx++)
        {
            const std::pair<T, uint64_t> deserialized_item = item_deserializer(buffer, current_position);
            deserialized.push_back(deserialized_item.first);
            current_position += deserialized_item.second;
        }
        deserialized.shrink_to_fit();
        // Figure out how many bytes were read
        const uint64_t bytes_read = current_position - current;
        return std::make_pair(deserialized, bytes_read);
    }

    template<typename Key, typename T, typename Compare, typename Allocator>
    inline uint64_t SerializeMap(const std::map<Key, T, Compare, Allocator>& map_to_serialize, std::vector<uint8_t>& buffer, const std::function<uint64_t(const Key&, std::vector<uint8_t>&)>& key_serializer, const std::function<uint64_t(const T&, std::vector<uint8_t>&)>& value_serializer)
    {
        const uint64_t start_buffer_size = buffer.size();
        // First, write a uint64_t size header
        const uint64_t size = (uint64_t)map_to_serialize.size();
        SerializeFixedSizePOD<uint64_t>(size, buffer);
        // Serialize the contained items
        typename std::map<Key, T, Compare, Allocator>::const_iterator itr;
        for (itr = map_to_serialize.begin(); itr != map_to_serialize.end(); ++itr)
        {
            SerializePair<Key, T>(*itr, buffer, key_serializer, value_serializer);
        }
        // Figure out how many bytes were written
        const uint64_t end_buffer_size = buffer.size();
        const uint64_t bytes_written = end_buffer_size - start_buffer_size;
        return bytes_written;
    }

    template<typename Key, typename T, typename Compare, typename Allocator>
    inline std::pair<std::map<Key, T, Compare, Allocator>, uint64_t> DeserializeMap(const std::vector<uint8_t>& buffer, const uint64_t current, const std::function<std::pair<Key, uint64_t>(const std::vector<uint8_t>&, const uint64_t)>& key_deserializer, const std::function<std::pair<T, uint64_t>(const std::vector<uint8_t>&, const uint64_t)>& value_deserializer)
    {
        // First, try to load the header
        assert(current < buffer.size());
        uint64_t current_position = current;
        // Load the header
        const std::pair<uint64_t, uint64_t> deserialized_size = DeserializeFixedSizePOD<uint64_t>(buffer, current_position);
        const uint64_t size = deserialized_size.first;
        current_position += deserialized_size.second;
        // Deserialize the items
        std::map<Key, T, Compare, Allocator> deserialized;
        for (uint64_t idx = 0; idx < size; idx++)
        {
            std::pair<std::pair<Key, T>, uint64_t> deserialized_pair = DeserializePair(buffer, current_position, key_deserializer, value_deserializer);
            deserialized.insert(deserialized_pair.first);
            current_position += deserialized_pair.second;
        }
        // Figure out how many bytes were read
        const uint64_t bytes_read = current_position - current;
        return std::make_pair(deserialized, bytes_read);
    }

    template<typename First, typename Second>
    inline uint64_t SerializePair(const std::pair<First, Second>& pair_to_serialize, std::vector<uint8_t>& buffer, const std::function<uint64_t(const First&, std::vector<uint8_t>&)>& first_serializer, const std::function<uint64_t(const Second&, std::vector<uint8_t>&)>& second_serializer)
    {
        const uint64_t start_buffer_size = buffer.size();
        uint64_t running_total = 0u;
        // Write each element of the pair into the buffer
        running_total += first_serializer(pair_to_serialize.first, buffer);
        running_total += second_serializer(pair_to_serialize.second, buffer);
        // Figure out how many bytes were written
        const uint64_t end_buffer_size = buffer.size();
        const uint64_t bytes_written = end_buffer_size - start_buffer_size;
        assert(bytes_written == running_total);
        return bytes_written;
    }

    template<typename First, typename Second>
    inline const std::pair<std::pair<First, Second>, uint64_t> DeserializePair(const std::vector<uint8_t>& buffer, const uint64_t current, const std::function<std::pair<First, uint64_t>(const std::vector<uint8_t>&, const uint64_t)>& first_deserializer, const std::function<std::pair<Second, uint64_t>(const std::vector<uint8_t>&, const uint64_t)>& second_deserializer)
    {
        assert(current < buffer.size());
        // Deserialize each item in the pair individually
        uint64_t current_position = current;
        const std::pair<First, uint64_t> deserialized_first = first_deserializer(buffer, current_position);
        current_position += deserialized_first.second;
        const std::pair<Second, uint64_t> deserialized_second = second_deserializer(buffer, current_position);
        current_position += deserialized_second.second;
        // Build the resulting pair
        // TODO: Why can't I used make_pair here?
        const std::pair<First, Second> deserialized(deserialized_first.first, deserialized_second.first);
        // Figure out how many bytes were read
        const uint64_t bytes_read = current_position - current;
        return std::make_pair(deserialized, bytes_read);
    }

    inline void ConditionalPrint(const std::string& msg, const int32_t msg_level, const int32_t print_level)
    {
        if (unlikely(msg_level <= print_level))
        {
            std::cout << "[" << msg_level << "/" << print_level << "] " << msg << std::endl;
        }
    }

    inline bool CheckAllStringsForSubstring(const std::vector<std::string>& strings, const std::string& substring)
    {
        for (size_t idx = 0; idx < strings.size(); idx++)
        {
            const std::string& candidate_string = strings[idx];
            const size_t found = candidate_string.find(substring);
            if (found == std::string::npos)
            {
                return false;
            }
        }
        return true;
    }

    template <typename Key, typename Value, typename Compare=std::less<Key>, typename Allocator=std::allocator<std::pair<const Key, Value>>>
    inline std::vector<Key> GetKeys(const std::map<Key, Value, Compare, Allocator>& map)
    {
        std::vector<Key> keys;
        keys.reserve(map.size());
        typename std::map<Key, Value, Compare, Allocator>::const_iterator itr;
        for (itr = map.begin(); itr != map.end(); ++itr)
        {
            const Key cur_key = itr->first;
            keys.push_back(cur_key);
        }
        keys.shrink_to_fit();
        return keys;
    }

    template <typename Key, typename Value, typename Compare=std::less<Key>, typename Allocator=std::allocator<std::pair<const Key, Value>>>
    inline std::vector<std::pair<const Key, Value>, Allocator> GetKeysAndValues(const std::map<Key, Value, Compare, Allocator>& map)
    {
        std::vector<std::pair<const Key, Value>, Allocator> keys_and_values;
        keys_and_values.reserve(map.size());
        typename std::map<Key, Value, Compare, Allocator>::const_iterator itr;
        for (itr = map.begin(); itr != map.end(); ++itr)
        {
            const std::pair<Key, Value> cur_pair(itr->first, itr->second);
            keys_and_values.push_back(cur_pair);
        }
        keys_and_values.shrink_to_fit();
        return keys_and_values;
    }

    template <typename Key, typename Value, typename Compare=std::less<Key>, typename Allocator=std::allocator<std::pair<const Key, Value>>>
    inline std::map<Key, Value, Compare, Allocator> MakeFromKeysAndValues(const std::vector<std::pair<const Key, Value>, Allocator>& keys_and_values)
    {
        std::map<Key, Value, Compare, Allocator> map;
        for (size_t idx = 0; idx < keys_and_values.size(); idx++)
        {
            const std::pair<Key, Value>& cur_pair = keys_and_values[idx];
            map[cur_pair.first] = cur_pair.second;
        }
        return map;
    }

    template <typename Key, typename Value, typename Compare=std::less<Key>, typename KeyVectorAllocator=std::allocator<Key>, typename ValueVectorAllocator=std::allocator<Value>, typename PairAllocator=std::allocator<std::pair<const Key, Value>>>
    inline std::map<Key, Value, Compare, PairAllocator> MakeFromKeysAndValues(const std::vector<Key, KeyVectorAllocator>& keys, const std::vector<Value, ValueVectorAllocator>& values)
    {
        assert(keys.size() == values.size());
        std::map<Key, Value, Compare, PairAllocator> map;
        for (size_t idx = 0; idx < keys.size(); idx++)
        {
            const Key& cur_key = keys[idx];
            const Value& cur_value = values[idx];
            map[cur_key] = cur_value;
        }
        return map;
    }
}

#endif // ARC_HELPERS_HPP

#pragma once

#include <type_traits>
#include <tuple>

namespace NumericConvertions
{
	template<int32_t Min, int32_t Max, typename T>
	constexpr inline T limiter(const T& val)
	{
		static_assert(std::is_arithmetic<T>::value, "Argument type isn't arithmethic");

		return val > Min
			? val < Max
				? val
				: Max
			: Min;
	}

	template<typename T>
	inline T limiter(const T& min, const T& max, const T& val)
	{
		return val > min
			? val < max
				? val
				: max
			: min;
	}

	template<int32_t Min, int32_t Max, typename T>
	inline auto tuple_limiter(const T& val)
	{
		return std::make_tuple<T>(limiter<Min, Max>(val));
	}

	template<int32_t Min, int32_t Max, typename T, typename... Args>
	inline auto tuple_limiter(const T& val, const Args&... args)
	{
		return std::tuple_cat(
			tuple_limiter<Min, Max>(val),
			tuple_limiter<Min, Max>(args...));
	}

	template<typename T>
	class MinMaxProvider
	{
		static_assert(std::is_arithmetic<T>::value, "Argument type isn't arithmethic");

	public:

		constexpr MinMaxProvider(const T& min, const T& max)
			: _min(min)
			, _max(max)
		{}

		constexpr T min() const
		{
			return _min;
		}

		constexpr T max() const
		{
			return _max;
		}

	private:

		T _min;
		T _max;
	};
	
	template<int32_t Min, int32_t Max,
			 typename T>
	constexpr inline bool IsInRange(const T& value)
	{
		return value >= Min && value <= Max;
	}

	template<typename T>
	inline bool IsInRange(const T& min, const T& max, const T& value)
	{
		return value >= min && value <= max;
	}

	template<int32_t MinCur, int32_t MaxCur,
			 int32_t MinNew, int32_t MaxNew,
			 typename T>
	constexpr inline T RangeTransform(const T& value)
	{
		static_assert(std::is_arithmetic<T>::value, "Argument type isn't arithmethic");

		constexpr double g = double(MaxCur * MinNew - MinCur * MaxNew) / double(MaxCur - MinCur);
		constexpr double k = double(MaxNew - MinNew) / double(MaxCur - MinCur);

		return static_cast<T>(limiter<MinCur, MaxCur>(value) * k + g);
	}

	template<typename T>
	inline T RangeTransform(const T& min_cur, const T& max_cur, const T& min_new, const T& max_new, const T& val)
	{
		static_assert(std::is_arithmetic<T>::value, "Argument type isn't arithmethic");

		return (limiter(min_cur, max_cur, val) - min_cur) * (max_new - min_new) / (max_cur - min_cur) + min_new;
	}

	template<class T>
	constexpr inline T sign(T param) 
	{ 
		static_assert(std::is_arithmetic<T>::value, "Argument type isn't arithmethic");

		return 
			static_cast<T>(
				param > 0
					? 1
					: param < 0
						? -1
						: 0); 
	}

	template<typename T>
	constexpr inline uint32_t UpBitsCount(T value)
	{
		static_assert(std::is_arithmetic<T>::value, "Argument type isn't arithmethic");

		uint32_t c = 0;   				// c accumulates the total bits set in v
		for (c = 0 ; value ; c++)
			value &= value - 1;  		// clear the least significant bit set

		return c;
	}

	template<typename T>
	inline int16_t compose_int16(T a_hbyte, T a_lbyte)
	{
		return (static_cast<int16_t>(a_hbyte) << 8U) | (static_cast<uint8_t>(a_lbyte));
	}

	template<typename T>
	inline uint16_t compose_uint16(T a_hbyte, T a_lbyte)
	{
		return (static_cast<uint16_t>(a_hbyte) << 8U) | (static_cast<uint8_t>(a_lbyte));
	}

	template<typename T>
	inline int32_t compose_int32(T a_hword, T a_lword)
	{
		return (static_cast<int32_t >(a_hword) << 16U) | (static_cast<uint16_t>(a_lword));
	}

	template<typename T>
	inline uint32_t compose_uint32(T a_hword, T a_lword)
	{
		return (static_cast<uint32_t>(a_hword) << 16U) | (static_cast<uint16_t>(a_lword));
	}

	inline uint8_t HighByteOfHalfWord(uint16_t half_word)
	{
		return static_cast<uint8_t>(half_word >> 8);
	}

	inline uint8_t LowByteOfHalfWord(uint16_t half_word)
	{
		return static_cast<uint8_t>(half_word);
	}

	union UINT16_union
	{
		UINT16_union(uint16_t _val)
			: val(_val)
		{}

		uint16_t	val;
		uint8_t		bytes[2];
	};

	template<typename T>
	union SSerializer
	{
		SSerializer(T _t)
			: value(_t)
		{}

		SSerializer()
			: value(T())
		{}

		constexpr inline size_t Size()
		{
			return sizeof(T);
		}

		T			value;
		uint8_t		bytes[sizeof(T)];
	};

	using FloatShare	= SSerializer<float>;
	using DoubleShare	= SSerializer<double>;
	using Uint8Share	= SSerializer<uint8_t>;
	using Uint16Share	= SSerializer<uint16_t>;
	using Uint32Share	= SSerializer<uint32_t>;
	using Uint64Share	= SSerializer<uint64_t>;

} // end NumericConversion namespace
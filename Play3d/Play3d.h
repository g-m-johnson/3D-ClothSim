///////////////////////////////////////////////////////////////////////////
//      Copyright (C) Sumo Digital Ltd. All rights reserved.
///////////////////////////////////////////////////////////////////////////

#define PLAY3D_VERSION "v1.0.0-70-g187e357"
#define PLAY3D_BUILD_TIME "2024-01-18 10:20:49.214740"

#pragma once
#ifndef __PLAY3D__
#define __PLAY3D__

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif

//--------------------------------------------------------------------------------------------------------------
#include <cstdint>
#include <string>
#include <vector>
#include <unordered_map>
#include <algorithm>
#include <functional>
#include <atomic>
#include <windows.h>
#include <wincodec.h>
#include <dxgi1_3.h>
#include <d3d11_1.h>
#include <wrl/client.h>
#include <hidusage.h>
#include <xaudio2.h>
//--------------------------------------------------------------------------------------------------------------

//------------------------------------------------ Play3dApi.h ------------------------------------------------

//------------------------------------------------- TypesApi.h -------------------------------------------------

#define XAUDIO2_HELPER_FUNCTIONS

//---------------------------------------------- Debug/TraceApi.h ----------------------------------------------

namespace Play3d::Debug
{
	//! @brief Prints a string to the Debug Output Console.
	void Put(const char* pStr);

	//! @brief printf style output to the Debug Output Console.
	void Printf(const char* pFmtStr, ...);

	//! @brief printf style output with file and line reporting.
	void Tracef(const char* pStrFilename, unsigned int lineNum, const char* pFmtStr, ...);

	//! @brief Prints a horizontal line to the console.
	void HLine();
}

//! @brief Declares a function arguments as unused suppressing a compiler warning.
#define PLAY_UNUSED(x) (void)x

#ifdef _DEBUG
//! @brief Asserts a condition is true, reports the condition expression debug console.
#define PLAY_ASSERT(condition)                                                                                         \
	if (!(condition))                                                                                                  \
	{                                                                                                                  \
		Play3d::Debug::Tracef(__FILE__, __LINE__, "PLAY_ASSERT(%s) FAILED : ", #condition);                            \
		__debugbreak();                                                                                                \
	}

//! @brief Asserts a condition is true, reports the condition expression and a user formatted message.
#define PLAY_ASSERT_MSG(condition, fmt, ...)                                                                           \
	if (!(condition))                                                                                                  \
	{                                                                                                                  \
		Play3d::Debug::Tracef(__FILE__, __LINE__, "PLAY_ASSERT(%s) FAILED : "##fmt, #condition, __VA_ARGS__);          \
		__debugbreak();                                                                                                \
	}
#else
#define PLAY_ASSERT(condition)
#define PLAY_ASSERT_MSG(condition, fmt, ...)
#endif
#ifndef WIN32_LEAN_AND_MEAN
#error WIN32_LEAN_AND_MEAN must be defined
#endif
#ifndef NOMINMAX
#error NOMINMAX must be defined
#endif

#pragma warning(disable : 4100)
#pragma warning(disable : 4189)
#pragma warning(disable : 4201)

#define PLAY_NONCOPYABLE(classname)                                                                                    \
	classname(const classname&) = delete;                                                                              \
	classname& operator=(const classname&) = delete;

#define PLAY_SINGLETON_INTERFACE(classname)                                                                            \
public:                                                                                                                \
	static classname& Instance()                                                                                       \
	{                                                                                                                  \
		return *ms_pInstance;                                                                                          \
	}                                                                                                                  \
	static void Initialise()                                                                                           \
	{                                                                                                                  \
		if (!ms_pInstance)                                                                                             \
		{                                                                                                              \
			ms_pInstance = new classname;                                                                              \
		}                                                                                                              \
	}                                                                                                                  \
	static void Destroy()                                                                                              \
	{                                                                                                                  \
		delete ms_pInstance;                                                                                           \
		ms_pInstance = nullptr;                                                                                        \
	}                                                                                                                  \
                                                                                                                       \
private:                                                                                                               \
	static classname* ms_pInstance;

#define PLAY_SINGLETON_IMPL(classname) classname* classname::ms_pInstance = nullptr;

#define PLAY_DEFINE_RESOURCE_PLACEHOLDER(classname)                                                                    \
	class classname;                                                                                                   \
	using classname##Id = IdKey<classname>;                                                                            \
	struct classname##Desc                                                                                             \
	{                                                                                                                  \
	};                                                                                                                 \
	class classname                                                                                                    \
	{                                                                                                                  \
	public:                                                                                                            \
		classname(const classname##Desc& rDesc)                                                                        \
		{}                                                                                                             \
	};

#define PLAY_SAFE_RELEASE(ptr)                                                                                         \
	if (ptr)                                                                                                           \
	{                                                                                                                  \
		ptr->Release();                                                                                                \
		ptr = nullptr;                                                                                                 \
	}
#define PLAY_SAFE_DELETE(ptr)                                                                                          \
	if (ptr)                                                                                                           \
	{                                                                                                                  \
		delete ptr;                                                                                                    \
		ptr = nullptr;                                                                                                 \
	}
#define PLAY_SAFE_DELETE_ARRAY(ptr)                                                                                    \
	if (ptr)                                                                                                           \
	{                                                                                                                  \
		delete[] ptr;                                                                                                  \
		ptr = nullptr;                                                                                                 \
	}

namespace Play3d
{
	//!-------------------------------------------------------------
	//! Fundamental Types

	using u8 = uint8_t;
	using u16 = uint16_t;
	using u32 = uint32_t;
	using u64 = uint64_t;

	using s8 = int8_t;
	using s16 = int16_t;
	using s32 = int32_t;
	using s64 = int64_t;

	using f32 = float;
	using f64 = double;

	//!-------------------------------------------------------------
	//! Error codes

	using result_t = int;
	enum Result : result_t
	{
		RESULT_QUIT = -2,
		RESULT_FAIL = -1,
		RESULT_OK = 0
	};

	//!-------------------------------------------------------------
	//! IdKey can be used to hold an type-safe id or index

	template <typename KeyType, typename ValueType = u32, ValueType kInvalidValue = ~0u>
	class IdKey
	{
	public:
		using key_type = KeyType;
		using value_type = ValueType;

		IdKey()
			: m_value(kInvalidValue)
		{}
		explicit IdKey(ValueType value)
			: m_value(value)
		{}
		IdKey(const IdKey& op)
			: m_value(op.m_value)
		{}
		IdKey& operator=(const IdKey& op)
		{
			m_value = op.m_value;
			return *this;
		}
		value_type GetValue() const { return m_value; }
		bool IsInvalid() const { return m_value == kInvalidValue; }
		bool IsValid() const { return m_value != kInvalidValue; }
		void Invalidate() { m_value = kInvalidValue; }
		bool operator==(IdKey rhs) const { return m_value == rhs.m_value; }
		bool operator!=(IdKey rhs) const { return m_value != rhs.m_value; }

	private:
		ValueType m_value;
	};

	//!-------------------------------------------------------------
	//! ComPtr is very useful for managing DirectX11 objects.
	template <class T>
	using ComPtr = Microsoft::WRL::ComPtr<T>;
};

//------------------------------------------- Maths/MathConstants.h -------------------------------------------

namespace Play3d
{
	constexpr f32 kfHalfPi = 3.141592654f / 2.0f;
	constexpr f32 kfPi = 3.141592654f;
	constexpr f32 kfTwoPi = 6.283185307f;
	constexpr f32 kfRoot2Over2 = 0.707106781f;
}

//--------------------------------------------- Maths/ScalarMath.h ---------------------------------------------

namespace Play3d
{
	template <typename T>
	inline bool EqualTol(const T lhs, const T rhs, const T tolerance)
	{
		return std::abs(lhs - rhs) < tolerance;
	}
}

//--------------------------------------------- Maths/VectorMath.h ---------------------------------------------

namespace Play3d
{
	template <int N, typename T>
	struct TVector
	{
		T v[N];
	};

	template <typename T>
	struct TVector<2, T>
	{
		TVector() {}
		explicit TVector(T value)
			: x(value)
			, y(value)
		{}
		explicit TVector(T x, T y)
			: x(x)
			, y(y)
		{}
		union {
			T v[2];
			struct
			{
				T x;
				T y;
			};
		};
	};

	template <typename T>
	struct TVector<3, T>
	{
		TVector() {}
		explicit TVector(T value)
			: x(value)
			, y(value)
			, z(value)
		{}
		explicit TVector(T x, T y, T z)
			: x(x)
			, y(y)
			, z(z)
		{}
		explicit TVector(const TVector<2, T>& v, T z)
			: x(v.x)
			, y(v.y)
			, z(z)
		{}

		TVector<2, T> xy() const { return TVector<2, T>(x, y); }

		union {
			T v[3];
			struct
			{
				T x;
				T y;
				T z;
			};
		};
	};

	template <typename T>
	struct TVector<4, T>
	{
		TVector() {}
		explicit TVector(T value)
			: x(value)
			, y(value)
			, z(value)
			, w(value)
		{}
		explicit TVector(T x, T y, T z, T w)
			: x(x)
			, y(y)
			, z(z)
			, w(w)
		{}
		explicit TVector(const TVector<3, T>& v, T w)
			: x(v.x)
			, y(v.y)
			, z(v.z)
			, w(w)
		{}

		TVector<2, T> xy() const { return TVector<2, T>(x, y); }
		TVector<3, T> xyz() const { return TVector<3, T>(x, y, z); }

		union {
			T v[4];
			struct
			{
				T x;
				T y;
				T z;
				T w;
			};
		};
	};

	using Vector2f = TVector<2, float>;
	using Vector3f = TVector<3, float>;
	using Vector4f = TVector<4, float>;

	template <int N, typename T>
	inline TVector<N, T> operator+(const TVector<N, T>& lhs, const TVector<N, T>& rhs)
	{
		TVector<N, T> ret;
		for (int i = 0; i < N; ++i)
		{
			ret.v[i] = lhs.v[i] + rhs.v[i];
		}
		return ret;
	}

	template <int N, typename T>
	inline TVector<N, T>& operator+=(TVector<N, T>& lhs, const TVector<N, T>& rhs)
	{
		for (int i = 0; i < N; ++i)
		{
			lhs.v[i] += rhs.v[i];
		}
		return lhs;
	}

	template <int N, typename T>
	inline TVector<N, T> operator-(const TVector<N, T>& lhs, const TVector<N, T>& rhs)
	{
		TVector<N, T> ret;
		for (int i = 0; i < N; ++i)
		{
			ret.v[i] = lhs.v[i] - rhs.v[i];
		}
		return ret;
	}

	template <int N, typename T>
	inline TVector<N, T>& operator-=(TVector<N, T>& lhs, const TVector<N, T>& rhs)
	{
		for (int i = 0; i < N; ++i)
		{
			lhs.v[i] -= rhs.v[i];
		}
		return lhs;
	}

	template <int N, typename T>
	inline TVector<N, T> operator-(const TVector<N, T>& op)
	{
		TVector<N, T> ret;
		for (int i = 0; i < N; ++i)
		{
			ret.v[i] = -op.v[i];
		}
		return ret;
	}

	template <int N, typename T>
	inline TVector<N, T> operator*(const TVector<N, T>& lhs, const TVector<N, T>& rhs)
	{
		TVector<N, T> ret;
		for (int i = 0; i < N; ++i)
		{
			ret.v[i] = lhs.v[i] * rhs.v[i];
		}
		return ret;
	}

	template <int N, typename T>
	inline TVector<N, T> operator/(const TVector<N, T>& lhs, const TVector<N, T>& rhs)
	{
		TVector<N, T> ret;
		for (int i = 0; i < N; ++i)
		{
			ret.v[i] = lhs.v[i] / rhs.v[i];
		}
		return ret;
	}

	template <int N, typename T>
	inline TVector<N, T> operator*(const TVector<N, T>& lhs, const T rhs)
	{
		TVector<N, T> ret;
		for (int i = 0; i < N; ++i)
		{
			ret.v[i] = lhs.v[i] * rhs;
		}
		return ret;
	}

	template <int N, typename T>
	inline TVector<N, T> operator*(const T lhs, const TVector<N, T>& rhs)
	{
		return rhs * lhs;
	}

	template <int N, typename T>
	inline TVector<N, T> operator/(const TVector<N, T>& lhs, const T rhs)
	{
		return lhs * (T(1) / rhs);
	}

	template <int N, typename T>
	inline TVector<N, T> operator/(const T lhs, const TVector<N, T>& rhs)
	{
		TVector<N, T> ret;
		for (int i = 0; i < N; ++i)
		{
			ret.v[i] = lhs / rhs.v[i];
		}
		return ret;
	}

	template <int N, typename T>
	inline T dot(const TVector<N, T>& lhs, const TVector<N, T>& rhs)
	{
		T ret = 0.f;
		for (int i = 0; i < N; ++i)
		{
			ret += lhs.v[i] * rhs.v[i];
		}
		return ret;
	}

	template <int N, typename T>
	inline T lengthSqr(const TVector<N, T>& v)
	{
		return dot(v, v);
	}

	template <int N, typename T>
	inline T length(const TVector<N, T>& v)
	{
		return sqrt(dot(v, v));
	}

	template <int N, typename T>
	inline TVector<N, T> normalize(const TVector<N, T>& v)
	{
		return v / length(v);
	}

	template <typename T>
	inline TVector<3, T> cross(const TVector<3, T>& lhs, const TVector<3, T>& rhs)
	{
		return TVector<3, T>(lhs.y * rhs.z - lhs.z * rhs.y,
							 lhs.z * rhs.x - lhs.x * rhs.z,
							 lhs.x * rhs.y - lhs.y * rhs.x);
	}

	template <int N, typename T>
	inline bool operator==(const TVector<N, T>& lhs, const TVector<N, T>& rhs)
	{
		TVector<N, T> ret;
		for (int i = 0; i < N; ++i)
		{
			if (lhs.v[i] != rhs.v[i])
				return false;
		}
		return true;
	}

	template <int N, typename T>
	inline bool operator!=(const TVector<N, T>& lhs, const TVector<N, T>& rhs)
	{
		return !(lhs == rhs);
	}

	template <int N, typename T>
	inline bool EqualTol(const TVector<N, T>& lhs, const TVector<N, T>& rhs, const T tolerance)
	{
		TVector<N, T> ret;
		for (int i = 0; i < N; ++i)
		{
			if (std::abs(lhs.v[i] - rhs.v[i]) > tolerance)
			{
				return false;
			}
		}
		return true;
	}
}

//--------------------------------------------- Maths/MatrixMath.h ---------------------------------------------

namespace Play3d
{

	template <int ROWS, int COLUMNS, typename T>
	struct TMatrix
	{
		T m[COLUMNS][ROWS];
	};

	template <typename T>
	struct TMatrix<2, 2, T>
	{
		using VectorType = TVector<2, T>;
		TMatrix() {}
		explicit TMatrix(const VectorType& column0, const VectorType& column1)
			: m_column{column0, column1}
		{}
		union {
			T m[2][2];
			VectorType m_column[2];
		};
	};

	template <typename T>
	struct TMatrix<3, 3, T>
	{
		using VectorType = TVector<3, T>;
		TMatrix() {}
		explicit TMatrix(const VectorType& column0, const VectorType& column1, const VectorType& column2)
			: m_column{column0, column1, column2}
		{}

		TMatrix<2, 2, T> Upper2x2() const { return TMatrix<2, 2, T>(m_column[0].xy(), m_column[1].xy()); }

		union {
			T m[3][3];
			VectorType m_column[3];
		};
	};

	template <typename T>
	struct TMatrix<4, 4, T>
	{
		using VectorType = TVector<4, T>;
		TMatrix() {}
		explicit TMatrix(const VectorType& column0, const VectorType& column1, const VectorType& column2,
						 const VectorType& column3)
			: m_column{column0, column1, column2, column3}
		{}

		explicit TMatrix(const TMatrix<3, 3, T>& M, const TVector<3, T> v)
			: m_column{VectorType(M.m_column[0], 0),
					   VectorType(M.m_column[1], 0),
					   VectorType(M.m_column[2], 0),
					   VectorType(v, T(1))}
		{}

		TMatrix<3, 3, T> Upper3x3() const
		{
			return TMatrix<3, 3, T>(m_column[0].xyz(), m_column[1].xyz(), m_column[2].xyz());
		}

		union {
			T m[4][4];
			VectorType m_column[4];
		};
	};

	using Matrix2x2f = TMatrix<2, 2, f32>;
	using Matrix3x3f = TMatrix<3, 3, f32>;
	using Matrix4x4f = TMatrix<4, 4, f32>;

	template <int ROWS, int COLUMNS, typename T>
	inline TMatrix<ROWS, COLUMNS, T> operator+(const TMatrix<ROWS, COLUMNS, T>& lhs,
											   const TMatrix<ROWS, COLUMNS, T>& rhs)
	{
		TMatrix<ROWS, COLUMNS, T> ret;
		for (int i = 0; i < COLUMNS; ++i)
			for (int j = 0; j < ROWS; ++j)
			{
				ret.v[i][j] = lhs.v[i][j] + rhs.v[i][j];
			}
		return ret;
	}

	template <int ROWS, int COLUMNS, typename T>
	inline TMatrix<ROWS, COLUMNS, T> operator-(const TMatrix<ROWS, COLUMNS, T>& lhs,
											   const TMatrix<ROWS, COLUMNS, T>& rhs)
	{
		TMatrix<ROWS, COLUMNS, T> ret;
		for (int i = 0; i < COLUMNS; ++i)
			for (int j = 0; j < ROWS; ++j)
			{
				ret.v[i][j] = lhs.v[i][j] - rhs.v[i][j];
			}
		return ret;
	}

	template <int M, int N, int P, typename T>
	inline TMatrix<M, P, T> operator*(const TMatrix<M, N, T>& lhs, const TMatrix<N, P, T>& rhs)
	{
		TMatrix<M, P, T> ret;
		for (int i = 0; i < M; ++i)
		{
			for (int j = 0; j < P; ++j)
			{
				T t = 0;
				for (int k = 0; k < N; ++k)
				{
					t += lhs.m[k][i] * rhs.m[j][k];
				}
				ret.m[j][i] = t;
			}
		}
		return ret;
	}

	template <int ROWS, int COLUMNS, typename T>
	inline TVector<ROWS, T> Transform(const TMatrix<ROWS, COLUMNS, T>& m, const TVector<ROWS, T>& v)
	{
		TVector<ROWS, T> ret;

		for (int i = 0; i < ROWS; ++i)
		{
			ret.v[i] = T(0);
			for (int j = 0; j < COLUMNS; ++j)
			{
				ret.v[i] += m.m[j][i] * v.v[j];
			}
		}
		return ret;
	}

	template <int ROWS, int COLUMNS, typename T>
	inline TMatrix<COLUMNS, ROWS, T> Transpose(const TMatrix<ROWS, COLUMNS, T>& op)
	{
		TMatrix<COLUMNS, ROWS, T> ret;
		for (int i = 0; i < COLUMNS; ++i)
		{
			for (int j = 0; j < ROWS; ++j)
			{
				ret.m[j][i] = op.m[i][j];
			}
		}
		return ret;
	}

	template <int ROWS, int COLUMNS, typename T>
	inline void MatrixFill(TMatrix<ROWS, COLUMNS, T>& rMatOut, const T value)
	{
		for (int i = 0; i < COLUMNS; ++i)
			for (int j = 0; j < ROWS; ++j)
			{
				rMatOut.m[i][j] = value;
			}
	}

	template <int ROWS, int COLUMNS, typename T>
	inline void MatrixFillIdentity(TMatrix<ROWS, COLUMNS, T>& rMatOut)
	{
		for (int i = 0; i < COLUMNS; ++i)
			for (int j = 0; j < ROWS; ++j)
			{
				rMatOut.m[i][j] = (i == j) ? T(1) : 0;
			}
	}

	template <int DIM, typename T>
	inline TMatrix<DIM, DIM, T> MatrixIdentity()
	{
		TMatrix<DIM, DIM, T> mat;
		MatrixFillIdentity(mat);
		return mat;
	}

	template <typename T>
	TMatrix<4, 4, T> MatrixRotationX(const T theta)
	{
		T c = cos(theta);
		T s = sin(theta);

		return TMatrix<4, 4, T>(TVector<4, T>(1, 0, 0, 0),
								TVector<4, T>(0, c, s, 0),
								TVector<4, T>(0, -s, c, 0),
								TVector<4, T>(0, 0, 0, 1));
	}
	template <typename T>
	TMatrix<4, 4, T> MatrixRotationY(const T theta)
	{
		T c = cos(theta);
		T s = sin(theta);

		return TMatrix<4, 4, T>(TVector<4, T>(c, 0, -s, 0),
								TVector<4, T>(0, 1, 0, 0),
								TVector<4, T>(s, 0, c, 0),
								TVector<4, T>(0, 0, 0, 1));
	}
	template <typename T>
	TMatrix<4, 4, T> MatrixRotationZ(const T theta)
	{
		T c = cos(theta);
		T s = sin(theta);

		return TMatrix<4, 4, T>(TVector<4, T>(c, s, 0, 0),
								TVector<4, T>(-s, c, 0, 0),
								TVector<4, T>(0, 0, 1, 0),
								TVector<4, T>(0, 0, 0, 1));
	}
	template <typename T>
	TMatrix<4, 4, T> MatrixScale(const T x, const T y, const T z)
	{
		return TMatrix<4, 4, T>(TVector<4, T>(x, 0, 0, 0),
								TVector<4, T>(0, y, 0, 0),
								TVector<4, T>(0, 0, z, 0),
								TVector<4, T>(0, 0, 0, 1));
	}

	template <typename T>
	TMatrix<4, 4, T> MatrixScale(const TVector<3, T>& v)
	{
		return MatrixScale(v.x, v.y, v.z);
	}

	template <typename T>
	TMatrix<4, 4, T> MatrixTranslate(const T x, const T y, const T z)
	{
		return TMatrix<4, 4, T>(TVector<4, T>(1, 0, 0, 0),
								TVector<4, T>(0, 1, 0, 0),
								TVector<4, T>(0, 0, 1, 0),
								TVector<4, T>(x, y, z, 1));
	}

	template <typename T>
	TMatrix<4, 4, T> MatrixTranslate(const TVector<3, T>& v)
	{
		return MatrixTranslate(v.x, v.y, v.z);
	}

	template <typename T>
	TMatrix<4, 4, T> MatrixOrthoProjectLH(const T left, const T right, const T bottom, const T top, const T nearZ,
										  const T farZ)
	{
		T w = right - left;
		T h = top - bottom;

		return TMatrix<4, 4, T>(TVector<4, T>(T(2) / w, 0, 0, 0),
								TVector<4, T>(0, T(2) / h, 0, 0),
								TVector<4, T>(0, 0, T(1) / (farZ - nearZ), 0),
								TVector<4, T>(-(right + left) / w, -(top + bottom) / h, nearZ / (nearZ - farZ), 1));
	}

	template <typename T>
	TMatrix<4, 4, T> MatrixOrthoProjectRH(const T left, const T right, const T bottom, const T top, const T nearZ,
										  const T farZ)
	{
		T w = right - left;
		T h = top - bottom;

		return TMatrix<4, 4, T>(TVector<4, T>(T(2) / w, 0, 0, 0),
								TVector<4, T>(0, T(2) / h, 0, 0),
								TVector<4, T>(0, 0, -T(1) / (farZ - nearZ), 0),
								TVector<4, T>(-(right + left) / w, -(top + bottom) / h, nearZ / (nearZ - farZ), 1));
	}

	template <typename T>
	TMatrix<4, 4, T> MatrixPerspectiveProjectLH(const T fovY, const T aspect, const T nearZ, const T farZ)
	{
		T a = T(1) / tan(fovY * T(0.5));
		T b = farZ / (farZ - nearZ);

		return TMatrix<4, 4, T>(TVector<4, T>(a / aspect, 0, 0, 0),
								TVector<4, T>(0, a, 0, 0),
								TVector<4, T>(0, 0, b, 1),
								TVector<4, T>(0, 0, -nearZ * b, 0));
	}

	template <typename T>
	TMatrix<4, 4, T> MatrixPerspectiveProjectRH(const T fovY, const T aspect, const T nearZ, const T farZ)
	{
		T a = T(1) / tan(fovY * T(0.5));
		T b = farZ / (farZ - nearZ);

		return TMatrix<4, 4, T>(TVector<4, T>(a / aspect, 0, 0, 0),
								TVector<4, T>(0, a, 0, 0),
								TVector<4, T>(0, 0, -b, -1),
								TVector<4, T>(0, 0, -nearZ * b, 0));
	}

	template <typename T>
	TMatrix<4, 4, T> MatrixLookatLH(const TVector<3, T>& eye, const TVector<3, T>& target, const TVector<3, T>& up)
	{
		TVector<3, T> w = normalize(target - eye);
		TVector<3, T> u = normalize(cross(up, w));
		TVector<3, T> v = cross(w, u);

		TVector<3, T> negEye = -eye;

		return TMatrix<4, 4, T>(TVector<4, T>(u.x, v.x, w.x, 0),
								TVector<4, T>(u.y, v.y, w.y, 0),
								TVector<4, T>(u.z, v.z, w.z, 0),
								TVector<4, T>(dot(negEye, u), dot(negEye, v), dot(negEye, w), 1));
	}
	template <typename T>
	TMatrix<4, 4, T> MatrixLookatRH(const TVector<3, T>& eye, const TVector<3, T>& target, const TVector<3, T>& up)
	{
		TVector<3, T> w = -normalize(target - eye);
		TVector<3, T> u = normalize(cross(up, w));
		TVector<3, T> v = cross(w, u);

		TVector<3, T> negEye = -eye;

		return TMatrix<4, 4, T>(TVector<4, T>(u.x, v.x, w.x, 0),
								TVector<4, T>(u.y, v.y, w.y, 0),
								TVector<4, T>(u.z, v.z, w.z, 0),
								TVector<4, T>(dot(negEye, u), dot(negEye, v), dot(negEye, w), 1));
	}

	template <int ROWS, int COLUMNS, typename T>
	inline bool EqualTol(const TMatrix<ROWS, COLUMNS, T>& lhs, const TMatrix<ROWS, COLUMNS, T>& rhs, const T tolerance)
	{
		for (int i = 0; i < COLUMNS; ++i)
		{
			for (int j = 0; j < ROWS; ++j)
			{
				if (std::abs(lhs.m[i][j] - rhs.m[i][j]) > tolerance)
				{
					return false;
				}
			}
		}
		return true;
	}

	template <int ROWS, int COLUMNS, typename T>
	inline bool EqualTolPtr(const TMatrix<ROWS, COLUMNS, T>& lhs, const T* pB, const T tolerance)
	{
		const T* pA = static_cast<const T*>(&lhs.m[0][0]);
		for (u32 i = 0; i < (ROWS * COLUMNS); ++i)
		{
			if (std::abs(pA[i] - pB[i]) > tolerance)
			{
				return false;
			}
		}
		return true;
	}

	template <typename T>
	inline T det2x2(T a, T b, T c, T d)
	{
		return a * d - b * c;
	}

	template <typename T>
	inline T Determinant(const TMatrix<2, 2, T>& m)
	{
		return det2x2<T>(m.m[0][0], m.m[1][0], m.m[0][1], m.m[1][1]);
	}

	template <typename T>
	inline T Determinant(const TMatrix<3, 3, T>& m)
	{
		return m.m[0][0] * m.m[1][1] * m.m[2][2] + m.m[1][0] * m.m[2][1] * m.m[0][2] + m.m[2][0] * m.m[0][1] * m.m[1][2]
			   - m.m[0][0] * m.m[2][1] * m.m[1][2] - m.m[1][0] * m.m[0][1] * m.m[2][2]
			   - m.m[2][0] * m.m[1][1] * m.m[0][2];
	}

	template <typename T>
	inline TMatrix<2, 2, T> Inverse(const TMatrix<2, 2, T>& m)
	{
		T d = Determinant(m);
		PLAY_ASSERT_MSG(d != 0.f, "Zero determinant");

		T f = T(1) / d;

		return TMatrix<2, 2, T>(TVector<2, T>(m.m[1][1] * f, -m.m[0][1] * f),
								TVector<2, T>(-m.m[1][0] * f, m.m[0][0] * f));
	}

	template <typename T>
	inline TMatrix<3, 3, T> Inverse(const TMatrix<3, 3, T>& m)
	{
		T d = Determinant(m);
		PLAY_ASSERT_MSG(d != 0.f, "Zero determinant");

		T f = T(1) / d;

		T c00 = det2x2(m.m[1][1], m.m[2][1], m.m[1][2], m.m[2][2]) * f;
		T c10 = det2x2(m.m[0][1], m.m[2][1], m.m[0][2], m.m[2][2]) * -f;
		T c20 = det2x2(m.m[0][1], m.m[1][1], m.m[0][2], m.m[1][2]) * f;

		T c01 = det2x2(m.m[1][0], m.m[2][0], m.m[1][2], m.m[2][2]) * -f;
		T c11 = det2x2(m.m[0][0], m.m[2][0], m.m[0][2], m.m[2][2]) * f;
		T c21 = det2x2(m.m[0][0], m.m[1][0], m.m[0][2], m.m[1][2]) * -f;

		T c02 = det2x2(m.m[1][0], m.m[2][0], m.m[1][1], m.m[2][1]) * f;
		T c12 = det2x2(m.m[0][0], m.m[2][0], m.m[0][1], m.m[2][1]) * -f;
		T c22 = det2x2(m.m[0][0], m.m[1][0], m.m[0][1], m.m[1][1]) * f;

		return TMatrix<3, 3, T>(TVector<3, T>(c00, c10, c20),
								TVector<3, T>(c01, c11, c21),
								TVector<3, T>(c02, c12, c22));
	}

	template <typename T>
	inline TMatrix<4, 4, T> AffineInverse(const TMatrix<4, 4, T>& m)
	{
		TMatrix<3, 3, T> invUpper(Inverse(m.Upper3x3()));
		TVector<3, T> v = -Transform(invUpper, Vector3f(m.m[3][0], m.m[3][1], m.m[3][2]));
		return Matrix4x4f(invUpper, v);
	}

	template <typename T>
	inline TMatrix<4, 4, T> InversePerspectiveProjectRH(const TMatrix<4, 4, T>& m)
	{
		return TMatrix<4, 4, T>(TVector<4, T>(T(1)/m.m[0][0], 0, 0, 0),
								TVector<4, T>(0, T(1)/m.m[1][1], 0, 0),
								TVector<4, T>(0, 0, 0, T(1)/m.m[3][2]),
								TVector<4, T>(0, 0, T(-1), m.m[2][2] / m.m[3][2]));
	}
}

//------------------------------------------- Maths/QuaternionMath.h -------------------------------------------

namespace Play3d
{

	//! Quaternion
	template <typename T>
	struct TQuat
	{
		TQuat() {}

		explicit TQuat(T x, T y, T z, T w)
			: v(x, y, z)
			, w(w)
		{}

		explicit TQuat(const TVector<3, T>& v, T w)
			: v(v)
			, w(w)
		{}

		explicit TQuat(const TVector<4, T>& v)
			: v(v.xyz())
			, w(v.w)
		{}

		union {
			T values[4];
			struct
			{
				TVector<3, T> v;
				T w;
			};
		};
	};

	using Quatf = TQuat<float>;

	template <typename T>
	inline TQuat<T> operator+(const TQuat<T>& lhs, const TQuat<T>& rhs)
	{
		return TQuat<T>(lhs.v + rhs.v, lhs.w + rhs.w);
	}

	template <typename T>
	inline TQuat<T>& operator+=(TQuat<T>& lhs, const TQuat<T>& rhs)
	{
		lhs.v += rhs.v;
		lhs.w += rhs.w;
		return lhs;
	}

	template <typename T>
	inline TQuat<T> operator-(const TQuat<T>& lhs, const TQuat<T>& rhs)
	{
		return TQuat<T>(lhs.v - rhs.v, lhs.w - rhs.w);
	}

	template <typename T>
	inline TQuat<T> operator-(const TQuat<T>& rhs)
	{
		return TQuat<T>(-rhs.v, -rhs.w);
	}

	template <typename T>
	inline TQuat<T>& operator-=(TQuat<T>& lhs, const TQuat<T>& rhs)
	{
		lhs.v -= rhs.v;
		lhs.w -= rhs.w;
		return lhs;
	}

	template <typename T>
	inline TQuat<T> operator*(const TQuat<T>& lhs, const T rhs)
	{
		return TQuat<T>(lhs.v * rhs, lhs.w * rhs);
	}

	template <typename T>
	inline TQuat<T> operator*(const T lhs, const TQuat<T>& rhs)
	{
		return rhs * lhs;
	}

	template <typename T>
	inline TQuat<T> operator*(const TQuat<T>& lhs, const TQuat<T>& rhs)
	{
		return TQuat<T>(lhs.w * rhs.v + rhs.w * lhs.v + cross(lhs.v, rhs.v), lhs.w * rhs.w - dot(lhs.v, rhs.v));
	}

	template <typename T>
	inline TQuat<T>& operator*=(TQuat<T>& lhs, const TQuat<T>& rhs)
	{
		return lhs = lhs * rhs;
	}

	template <typename T>
	inline T dot(const TQuat<T>& lhs, const TQuat<T>& rhs)
	{
		return lhs.v.x * rhs.v.x + lhs.v.y * rhs.v.y + lhs.v.z * rhs.v.z + lhs.w * rhs.w;
	}
	template <typename T>
	inline TQuat<T> conj(const TQuat<T>& q)
	{
		return TQuat<T>(-q.v, q.w);
	}
	template <typename T>
	inline TQuat<T> normalize(const TQuat<T>& q)
	{
		T fInvNorm = T(1) / sqrt(q.w * q.w + dot(q.v, q.v));
		return TQuat<T>(q.v * fInvNorm, q.w * fInvNorm);
	}

	template <typename T>
	inline TQuat<T> lerp(const TQuat<T>& a, const TQuat<T>& b, T t)
	{
		return (T(1) - t) * a + t * b;
	}

	template <typename T>
	inline TQuat<T> slerp(const TQuat<T>& a, const TQuat<T>& b, T t)
	{

		T cOmega = dot(a, b);
		TQuat<T> nb;
		if (cOmega < T(0))
		{
			cOmega = -cOmega;
			nb = -b;
		}
		else
		{
			nb = b;
		}

		T k0, k1;
		if (cOmega > T(0.9999))
		{
			k0 = T(1) - t;
			k1 = t;
		}
		else
		{
			T sOmega = sqrt(T(1) - cOmega * cOmega);
			T w =  atan2(sOmega, cOmega);
			T rw = T(1) / w;
			k0 = sin((T(1) - t) * w) * rw;
			k1 = sin(t * w) * rw;
		}
		return normalize( k0*a + k1*nb );
	}

	template <typename T>
	inline TQuat<T> QuaternionIdentity()
	{
		return TQuat<T>(0, 0, 0, T(1));
	}

	//! Construct a Quaternion from Angle and Axis Vector
	//! @param axis : unit vector around which to rotate
	//! @param theta : angle to rotate in radians
	template <typename T>
	inline TQuat<T> QuaternionFromAxisAngle(const TVector<3,T> axis, T theta)
	{
		return TQuat<T>(axis * sin(theta / T(2)), cos(theta / T(2)));
	}

	//! Construct a Quaternion from Euler Angles.
	//! Typical aerospace ordering yaw then pitch then roll.
	//! @param yaw : rotate around Y
	//! @param pitch : rotate around X
	//! @param roll : rotate around Z
	template <typename T>
	inline TQuat<T> QuaternionFromEuler(T yaw, T pitch, T roll)
	{
		const T sYaw = sin(yaw / T(2));
		const T cYaw = cos(yaw / T(2));

		const T sPitch = sin(pitch / T(2));
		const T cPitch = cos(pitch / T(2));

		const T sRoll = sin(roll / T(2));
		const T cRoll = cos(roll / T(2));

		return TQuat<T>(cYaw * sPitch * cRoll + sYaw * cPitch * sRoll,
						sYaw * cPitch * cRoll - cYaw * sPitch * sRoll,
						cYaw * cPitch * sRoll - sYaw * sPitch * cRoll,
						cYaw * cPitch * cRoll + sYaw * sPitch * sRoll);
	}

	template <typename T>
	inline TMatrix<4, 4, T> MatrixFromQuaternion(const TQuat<T>& q)
	{
		const T x = q.v.x;
		const T y = q.v.y;
		const T z = q.v.z;
		const T w = q.w;

		const T x2 = T(2) * x * x;
		const T y2 = T(2) * y * y;
		const T z2 = T(2) * z * z;

		const T xy = T(2) * x * y;
		const T xz = T(2) * x * z;

		const T yz = T(2) * y * z;

		const T wx = T(2) * w * x;
		const T wy = T(2) * w * y;
		const T wz = T(2) * w * z;

		return TMatrix<4, 4, T>(TVector<4, T>(T(1) - y2 - z2, xy + wz, xz - wy, T(0)),
								TVector<4, T>(xy - wz, T(1) - x2 - z2, yz + wx, T(0)),
								TVector<4, T>(xz + wy, yz - wx, T(1) - x2 - y2, T(0)),
								TVector<4, T>(T(0), T(0), T(0), T(1))
		);
	}

	template <typename T>
	inline bool operator==(const TQuat<T>& lhs, const TQuat<T>& rhs)
	{
		return lhs.v == rhs.v && lhs.w == rhs.w;
	}

	template <typename T>
	inline bool operator!=(const TQuat<T>& lhs, const TQuat<T>& rhs)
	{
		return !(lhs == rhs);
	}

	template <typename T>
	inline bool EqualTol(const TQuat<T>& lhs, const TQuat<T>& rhs, const T tolerance)
	{
		if (std::abs(lhs.w - rhs.w) > tolerance)
		{
			return false;
		}
		return EqualTol(lhs.v, rhs.v, tolerance);
	}

	template <typename T>
	inline bool EqualTolPtr(const TQuat<T>& lhs, const T* pB, const T tolerance)
	{
		const T* pA = static_cast<const T*>(&lhs.v.x);
		for (u32 i = 0; i < 4; ++i)
		{
			if (std::abs(pA[i] - pB[i]) > tolerance)
			{
				return false;
			}
		}
		return true;
	}

}

//------------------------------------------ Maths/IntersectionMath.h ------------------------------------------

namespace Play3d
{
	struct AABBMinMax
	{
		Vector3f vMin;
		Vector3f vMax;
	};

	struct AABBHalfSize
	{
		Vector3f vOrigin;
		Vector3f vHalfSize;
	};

	struct Ray
	{
		Vector3f vOrigin;
		Vector3f vDirection;
	};

	void ComputeBoundsFromPositions(const Vector3f* pPositions, size_t numElements, AABBMinMax& rBoundsOut);

	inline AABBHalfSize ConvertBounds(const AABBMinMax& in)
	{
		AABBHalfSize result;
		result.vHalfSize = (in.vMax - in.vMin) * 0.5f;
		result.vOrigin = in.vMin + result.vHalfSize;
		return result;
	}

	inline AABBMinMax ConvertBounds(const AABBHalfSize& in)
	{
		AABBMinMax result;
		result.vMin = in.vOrigin - in.vHalfSize;
		result.vMax = in.vOrigin + in.vHalfSize;
		return result;
	}
}
//------------------------------------------ Resources/ResourcesApi.h ------------------------------------------

namespace Play3d::Resources
{
	//! @brief The resource manager defines a singleton manager for a given resource type.
	//! Resources can be created and destroy through it.
	//! Any resource can be given an alias (name) that can be used to look it up.
	//! @tparam T
	template <typename T>
	class ResourceManager
	{
		ResourceManager() {}
		~ResourceManager() { ReleaseAll(); }

	public:
		//! @brief Returns the singleton instance.
		static ResourceManager& Instance()
		{
			static ResourceManager s_instance;
			return s_instance;
		}

		//! @brief Calls the constructor for the managed type.
		//! @param ...args Arguments forwarded to the parameterized constructor.
		//! @return a handle (id) to the created object.
		template <typename... ConstructorArgs>
		IdKey<T> Create(ConstructorArgs... args)
		{
			T* p = new T(args...);
			PLAY_ASSERT(p);
			if (!p)
			{
				return IdKey<T>();
			}
			if (m_freelist.empty())
			{
				u32 index = (u32)m_objects.size();
				m_objects.push_back(p);

				return IdKey<T>(index);
			}
			else
			{
				u32 index = m_freelist.back();
				m_freelist.pop_back();

				m_objects[index] = p;
				return IdKey<T>(index);
			}
		}

		//! @brief Releases (destroys) the object with the specified id.
		void Release(IdKey<T> id)
		{
			if (id.IsValid())
			{
				RemoveAliasesTo(id);

				u32 index = id.GetValue();
				T* p = m_objects.at(index);
				PLAY_ASSERT(p);
				;
				m_objects.at(index) = nullptr;
				m_freelist.push_back(index);
				delete p;
			}
		}

		//! @brief Removes aliases for the object with the specified id.
		void RemoveAliasesTo(IdKey<T> id)
		{
			if (id.IsValid())
			{
				for (auto it = m_namedObjectLUT.begin(); it != m_namedObjectLUT.end();)
				{
					if (it->second == id)
					{
						it = m_namedObjectLUT.erase(it);
					}
					else
					{
						++it;
					}
				}
			}
		}

		//! @brief Finds an object id associated with the an alias.
		//! @param name the name to search for
		//! @return an id, if the object is not found the id will have an invalid value.
		IdKey<T> Find(std::string_view name) const
		{
			auto it = m_namedObjectLUT.find(std::string(name));
			if (it != m_namedObjectLUT.end())
			{
				return it->second;
			}
			return IdKey<T>();
		}

		//! @brief Obtains a pointer to an object with the specified id.
		//! @return an object ptr, if the object is not found the ptr will be nullptr.
		T* GetPtr(IdKey<T> id) const
		{
			if (id.IsValid())
			{
				u32 index = id.GetValue();
				return m_objects.at(index);
			}
			return nullptr;
		}

		//! @brief Associates an alias (name) with a particular id.
		void AddAlias(std::string_view name, IdKey<T> id) { m_namedObjectLUT.insert({std::string(name), id}); }

		//! @brief Releases all managed objects.
		void ReleaseAll()
		{
			m_namedObjectLUT.clear();

			for (u32 i = 0; i < (u32)m_objects.size(); ++i)
			{
				if (m_objects[i])
				{
					delete m_objects[i];
					m_objects[i] = nullptr;
					m_freelist.push_back(i);
				}
			}
		}

	private:
		std::vector<T*> m_objects;
		std::vector<uint32_t> m_freelist;
		std::unordered_map<std::string, IdKey<T>> m_namedObjectLUT;
	};

	//! @brief Finds an object id associated with the an alias.
	//! @param name the name to search for
	//! @return an id, if the object is not found the id will have an invalid value.
	template <typename T>
	inline IdKey<T> FindAsset(std::string_view name)
	{
		return ResourceManager<T>::Instance().Find(name);
	}

	//! @brief Associates an alias (name) with a particular id.
	template <typename T>
	inline void AddAlias(std::string_view name, IdKey<T> id)
	{
		return ResourceManager<T>::Instance().AddAlias(name, id);
	}

	//! @brief Obtains a pointer to an object with the specified id.
	//! @return an object ptr, if the object is not found the ptr will be nullptr.
	template <typename T>
	inline T* GetPtr(IdKey<T> id)
	{
		return ResourceManager<T>::Instance().GetPtr(id);
	}

	//! @brief Calls the constructor for the managed type.
	//! @param ...args Arguments forwarded to the parameterized constructor.
	//! @return A handle (id) to the created object.
	template <typename T, typename... ConstructorArgs>
	inline IdKey<T> CreateAsset(ConstructorArgs... args)
	{
		return ResourceManager<T>::Instance().Create(args...);
	}

	class AsyncLoadingTask
	{
	};

	IdKey<AsyncLoadingTask> AsyncLoadAssets(std::string_view pathToAssetPack);

	bool AssetsLoaded(IdKey<AsyncLoadingTask> hAsyncLoad);

	result_t LoadAssets(std::string_view pathToAssetPack);
}

//---------------------------------------------- Input/InputApi.h ----------------------------------------------

namespace Play3d::Input
{
	enum class InputDevice
	{
		GAMEPAD_0,
		GAMEPAD_1,
		GAMEPAD_2,
		GAMEPAD_3,
	};

	enum class ButtonId
	{
		BUTTON_DPAD_UP,
		BUTTON_DPAD_DOWN,
		BUTTON_DPAD_LEFT,
		BUTTON_DPAD_RIGHT,
		BUTTON_X,
		BUTTON_Y,
		BUTTON_A,
		BUTTON_B,
		BUTTON_LEFT_SHOULDER,
		BUTTON_RIGHT_SHOULDER,
		BUTTON_LEFT_STICK,
		BUTTON_RIGHT_STICK,
		BUTTON_START,
		BUTTON_BACK,
		BUTTON_LEFT_TRIGGER,
		BUTTON_RIGHT_TRIGGER,

		BUTTON_COUNT
	};

	enum class AxisId
	{
		AXIS_LEFTSTICK_X,
		AXIS_LEFTSTICK_Y,
		AXIS_RIGHTSTICK_X,
		AXIS_RIGHTSTICK_Y,

		AXIS_LEFT_TRIGGER_PRESSURE,
		AXIS_RIGHT_TRIGGER_PRESSURE,

		AXIS_COUNT
	};

	struct MouseState
	{
		f32 m_x;
		f32 m_y;
		f32 m_deltaX;
		f32 m_deltaY;
		bool m_leftButton;
		bool m_middleButton;
		bool m_rightButton;
	};

	bool IsKeyPressed(u32 keycode);
	bool IsKeyDown(u32 keycode);
	bool IsButtonPressed(InputDevice device, ButtonId buttonId);
	bool IsButtonDown(InputDevice device, ButtonId buttonId);
	float GetAxis(InputDevice device, AxisId axisId);

	const MouseState& GetMouseState();
	void CaptureMouse(bool bEnable);
}

//---------------------------------------------- Audio/AudioApi.h ----------------------------------------------

//----------------------------------------------- Audio/Sound.h -----------------------------------------------

namespace Play3d::Audio
{
	class Sound;
	struct Voice;

	using SoundId = IdKey<Sound>;
	using VoiceId = IdKey<Voice>;

	struct SoundDesc
	{
		size_t m_sizeBytes = 0;
		const void* m_pData = nullptr;
		bool m_bLoop = false;
	};

	class Sound
	{
	public:
		Sound(const SoundDesc& rDesc);
		~Sound();

	private:
		friend class Audio_Impl;
		WAVEFORMATEX m_format;
		XAUDIO2_BUFFER m_buffer;
		void* m_pData;
		size_t m_sizeBytes;
	};

}

namespace Play3d::Audio
{
	SoundId LoadSoundFromFile(const char* filePath, bool bEnableLooping = false);
	VoiceId PlaySound(SoundId soundId, f32 fGain = 1.0f, f32 fPan = 0.5f);
	void StopSound(VoiceId voiceId);

	void* GetAudioDevice();
};

//------------------------------------------- Graphics/MeshBuilder.h -------------------------------------------

//--------------------------------------------- Graphics/Colour.h ---------------------------------------------

namespace Play3d
{
	class ColourValue
	{
	public:
		ColourValue()
			: m_value(0)
		{}

		explicit constexpr ColourValue(const u32 value)
			: m_value(value)
		{}

		explicit constexpr ColourValue(const u8 r, const u8 g, const u8 b, const u8 a = 255)
			: m_value(r | (g << 8) | (b << 16) | (a << 24))
		{}

		constexpr u32 as_u32() const { return m_value; }

		constexpr u32 with_alpha(u8 alpha) const { return (m_value & 0x00ffFFFF) | (alpha << 24); }

		constexpr void as_float_rgba_linear(f32 f[]) const
		{
			f[0] = f32((m_value)&0xFF) / 255.f;
			f[1] = f32((m_value >> 8) & 0xFF) / 255.f;
			f[2] = f32((m_value >> 16) & 0xFF) / 255.f;
			f[3] = f32((m_value >> 24) & 0xFF) / 255.f;
		}

		void as_float_rgba_srgb(f32 f[]) const
		{
			f[0] = pow(f32((m_value)&0xFF) / 255.f, 2.2f);
			f[1] = pow(f32((m_value >> 8) & 0xFF) / 255.f, 2.2f);
			f[2] = pow(f32((m_value >> 16) & 0xFF) / 255.f, 2.2f);
			f[3] = pow(f32((m_value >> 24) & 0xFF) / 255.f, 2.2f);
		}

	private:
		u32 m_value;
	};

	namespace Colour
	{
		constexpr ColourValue Aliceblue(240, 248, 255);
		constexpr ColourValue Antiquewhite(250, 235, 215);
		constexpr ColourValue Aqua(0, 255, 255);
		constexpr ColourValue Aquamarine(127, 255, 212);
		constexpr ColourValue Azure(240, 255, 255);
		constexpr ColourValue Beige(245, 245, 220);
		constexpr ColourValue Bisque(255, 228, 196);
		constexpr ColourValue Black(0, 0, 0);
		constexpr ColourValue Blanchedalmond(255, 235, 205);
		constexpr ColourValue Blue(0, 0, 255);
		constexpr ColourValue Blueviolet(138, 43, 226);
		constexpr ColourValue Brown(165, 42, 42);
		constexpr ColourValue Burlywood(222, 184, 135);
		constexpr ColourValue Cadetblue(95, 158, 160);
		constexpr ColourValue Chartreuse(127, 255, 0);
		constexpr ColourValue Chocolate(210, 105, 30);
		constexpr ColourValue Coral(255, 127, 80);
		constexpr ColourValue Cornflowerblue(100, 149, 237);
		constexpr ColourValue Cornsilk(255, 248, 220);
		constexpr ColourValue Crimson(220, 20, 60);
		constexpr ColourValue Cyan(0, 255, 255);
		constexpr ColourValue Darkblue(0, 0, 139);
		constexpr ColourValue Darkcyan(0, 139, 139);
		constexpr ColourValue Darkgoldenrod(184, 134, 11);
		constexpr ColourValue Darkgray(169, 169, 169);
		constexpr ColourValue Darkgreen(0, 100, 0);
		constexpr ColourValue Darkgrey(169, 169, 169);
		constexpr ColourValue Darkkhaki(189, 183, 107);
		constexpr ColourValue Darkmagenta(139, 0, 139);
		constexpr ColourValue Darkolivegreen(85, 107, 47);
		constexpr ColourValue Darkorange(255, 140, 0);
		constexpr ColourValue Darkorchid(153, 50, 204);
		constexpr ColourValue Darkred(139, 0, 0);
		constexpr ColourValue Darksalmon(233, 150, 122);
		constexpr ColourValue Darkseagreen(143, 188, 143);
		constexpr ColourValue Darkslateblue(72, 61, 139);
		constexpr ColourValue Darkslategray(47, 79, 79);
		constexpr ColourValue Darkslategrey(47, 79, 79);
		constexpr ColourValue Darkturquoise(0, 206, 209);
		constexpr ColourValue Darkviolet(148, 0, 211);
		constexpr ColourValue Deeppink(255, 20, 147);
		constexpr ColourValue Deepskyblue(0, 191, 255);
		constexpr ColourValue Dimgray(105, 105, 105);
		constexpr ColourValue Dimgrey(105, 105, 105);
		constexpr ColourValue Dodgerblue(30, 144, 255);
		constexpr ColourValue Firebrick(178, 34, 34);
		constexpr ColourValue Floralwhite(255, 250, 240);
		constexpr ColourValue Forestgreen(34, 139, 34);
		constexpr ColourValue Fuchsia(255, 0, 255);
		constexpr ColourValue Gainsboro(220, 220, 220);
		constexpr ColourValue Ghostwhite(248, 248, 255);
		constexpr ColourValue Gold(255, 215, 0);
		constexpr ColourValue Goldenrod(218, 165, 32);
		constexpr ColourValue Gray(128, 128, 128);
		constexpr ColourValue Green(0, 128, 0);
		constexpr ColourValue Greenyellow(173, 255, 47);
		constexpr ColourValue Grey(128, 128, 128);
		constexpr ColourValue Honeydew(240, 255, 240);
		constexpr ColourValue Hotpink(255, 105, 180);
		constexpr ColourValue Indianred(205, 92, 92);
		constexpr ColourValue Indigo(75, 0, 130);
		constexpr ColourValue Ivory(255, 255, 240);
		constexpr ColourValue Khaki(240, 230, 140);
		constexpr ColourValue Lavender(230, 230, 250);
		constexpr ColourValue Lavenderblush(255, 240, 245);
		constexpr ColourValue Lawngreen(124, 252, 0);
		constexpr ColourValue Lemonchiffon(255, 250, 205);
		constexpr ColourValue Lightblue(173, 216, 230);
		constexpr ColourValue Lightcoral(240, 128, 128);
		constexpr ColourValue Lightcyan(224, 255, 255);
		constexpr ColourValue Lightgoldenrodyellow(250, 250, 210);
		constexpr ColourValue Lightgray(211, 211, 211);
		constexpr ColourValue Lightgreen(144, 238, 144);
		constexpr ColourValue Lightgrey(211, 211, 211);
		constexpr ColourValue Lightpink(255, 182, 193);
		constexpr ColourValue Lightsalmon(255, 160, 122);
		constexpr ColourValue Lightseagreen(32, 178, 170);
		constexpr ColourValue Lightskyblue(135, 206, 250);
		constexpr ColourValue Lightslategray(119, 136, 153);
		constexpr ColourValue Lightslategrey(119, 136, 153);
		constexpr ColourValue Lightsteelblue(176, 196, 222);
		constexpr ColourValue Lightyellow(255, 255, 224);
		constexpr ColourValue Lime(0, 255, 0);
		constexpr ColourValue Limegreen(50, 205, 50);
		constexpr ColourValue Linen(250, 240, 230);
		constexpr ColourValue Magenta(255, 0, 255);
		constexpr ColourValue Maroon(128, 0, 0);
		constexpr ColourValue Mediumaquamarine(102, 205, 170);
		constexpr ColourValue Mediumblue(0, 0, 205);
		constexpr ColourValue Mediumorchid(186, 85, 211);
		constexpr ColourValue Mediumpurple(147, 112, 219);
		constexpr ColourValue Mediumseagreen(60, 179, 113);
		constexpr ColourValue Mediumslateblue(123, 104, 238);
		constexpr ColourValue Mediumspringgreen(0, 250, 154);
		constexpr ColourValue Mediumturquoise(72, 209, 204);
		constexpr ColourValue Mediumvioletred(199, 21, 133);
		constexpr ColourValue Midnightblue(25, 25, 112);
		constexpr ColourValue Mintcream(245, 255, 250);
		constexpr ColourValue Mistyrose(255, 228, 225);
		constexpr ColourValue Moccasin(255, 228, 181);
		constexpr ColourValue Navajowhite(255, 222, 173);
		constexpr ColourValue Navy(0, 0, 128);
		constexpr ColourValue Oldlace(253, 245, 230);
		constexpr ColourValue Olive(128, 128, 0);
		constexpr ColourValue Olivedrab(107, 142, 35);
		constexpr ColourValue Orange(255, 165, 0);
		constexpr ColourValue Orangered(255, 69, 0);
		constexpr ColourValue Orchid(218, 112, 214);
		constexpr ColourValue Palegoldenrod(238, 232, 170);
		constexpr ColourValue Palegreen(152, 251, 152);
		constexpr ColourValue Paleturquoise(175, 238, 238);
		constexpr ColourValue Palevioletred(219, 112, 147);
		constexpr ColourValue Papayawhip(255, 239, 213);
		constexpr ColourValue Peachpuff(255, 218, 185);
		constexpr ColourValue Peru(205, 133, 63);
		constexpr ColourValue Pink(255, 192, 203);
		constexpr ColourValue Plum(221, 160, 221);
		constexpr ColourValue Powderblue(176, 224, 230);
		constexpr ColourValue Purple(128, 0, 128);
		constexpr ColourValue Red(255, 0, 0);
		constexpr ColourValue Rosybrown(188, 143, 143);
		constexpr ColourValue Royalblue(65, 105, 225);
		constexpr ColourValue Saddlebrown(139, 69, 19);
		constexpr ColourValue Salmon(250, 128, 114);
		constexpr ColourValue Sandybrown(244, 164, 96);
		constexpr ColourValue Seagreen(46, 139, 87);
		constexpr ColourValue Seashell(255, 245, 238);
		constexpr ColourValue Sienna(160, 82, 45);
		constexpr ColourValue Silver(192, 192, 192);
		constexpr ColourValue Skyblue(135, 206, 235);
		constexpr ColourValue Slateblue(106, 90, 205);
		constexpr ColourValue Slategray(112, 128, 144);
		constexpr ColourValue Slategrey(112, 128, 144);
		constexpr ColourValue Snow(255, 250, 250);
		constexpr ColourValue Springgreen(0, 255, 127);
		constexpr ColourValue Steelblue(70, 130, 180);
		constexpr ColourValue Tan(210, 180, 140);
		constexpr ColourValue Teal(0, 128, 128);
		constexpr ColourValue Thistle(216, 191, 216);
		constexpr ColourValue Tomato(255, 99, 71);
		constexpr ColourValue Turquoise(64, 224, 208);
		constexpr ColourValue Violet(238, 130, 238);
		constexpr ColourValue Wheat(245, 222, 179);
		constexpr ColourValue White(255, 255, 255);
		constexpr ColourValue Whitesmoke(245, 245, 245);
		constexpr ColourValue Yellow(255, 255, 0);
		constexpr ColourValue Yellowgreen(154, 205, 50);
	}
}

//---------------------------------------------- Graphics/Mesh.h ----------------------------------------------

namespace Play3d::Graphics
{
	class Mesh;
	using MeshId = IdKey<Mesh>;

	//! @brief StreamType maps to shader input semantics.
	//! Common Vertex Attributes match with internal shaders.
	//! Custom Semantics available in your own custom shaders.
	//! Index buffer has its own specialized StreamType.
	enum class StreamType
	{
		INDEX,
		POSITION,
		COLOUR,
		NORMAL,
		UV,
		TANGENT,
		JOINTID,
		JOINTWEIGHT,
		USER0,
		USER1,
		USER2,
		USER3,
		USER4,

		//! Extend as required
		//! Counter
		COUNT,
	};

	enum class MeshTopology
	{
		POINT_LIST,
		LINE_LIST,
		LINE_STRIP,
		TRI_LIST,
		TRI_STRIP,
	};

	namespace StreamFlags
	{
		enum Type
		{
			NONE = 0,
			DYNAMIC_STREAM,
			INSTANCE_STREAM
		};
	}
	struct StreamInfo
	{
		StreamType m_type = StreamType::POSITION;
		void* m_pData = nullptr;
		size_t m_dataSize = 0;
		u32 m_flags = StreamFlags::NONE;
		u32 m_stride = 0;
		u32 m_offset = 0;
	};

	//! @brief Represents a sub-range of Mesh data.
	//! This is typically used for LODs or Multi-material meshes.
	struct SubmeshDesc
	{
		u32 m_userId;
		u32 m_elementOffset;
		u32 m_elementSize;
	};

	//! @brief Describes a mesh
	struct MeshDesc
	{
		StreamInfo* m_pStreams = nullptr;
		u32 m_streamCount = 0;
		u32 m_indexCount = 0;
		u32 m_vertexCount = 0;
		MeshTopology m_topology = MeshTopology::TRI_LIST;
	};

	class Mesh
	{
	public:
		Mesh(const MeshDesc& rDesc);
		~Mesh();
		void AddStream(const StreamInfo& info);
		void Bind(ID3D11DeviceContext* pDC) const;

		const AABBMinMax& GetBounds() const { return m_bounds; }
		u32 GetVertexCount() const { return m_vertexCount; }
		u32 GetIndexCount() const { return m_indexCount; }

		void AddSubmesh(const SubmeshDesc& rSubMesh) { m_subMeshes.push_back(rSubMesh); }
		const SubmeshDesc& GetSubmesh(u32 index) const { return m_subMeshes.at(index); }

		ID3D11Buffer* GetStreamBufferPtr(int st) {return m_streamBuffers[st]; }

	private:
		friend class Graphics_Impl;
		std::vector<StreamInfo> m_streamInfos;
		std::vector<SubmeshDesc> m_subMeshes;
		std::vector<ID3D11Buffer*> m_streamBuffers;
		std::vector<UINT> m_strides;
		std::vector<UINT> m_offsets;
		ID3D11Buffer* m_pIndexBuffer;

		AABBMinMax m_bounds;

		u32 m_indexCount;
		u32 m_vertexCount;

		D3D11_PRIMITIVE_TOPOLOGY m_topology;

		bool m_bIsSkinnedMesh;
	};
}

namespace Play3d::Graphics
{
	//! @brief A helper class for building 3d geometry.
	//! Use 'Add' member functions to generate geometry then call CreateMesh the finalize the mesh.
	class MeshBuilder
	{
	public:
		//! @brief Adds an individual unconnected vertex.
		//! @returns vertex index.
		u32 AddVertex(const Vector3f& position, const Vector3f& normal, const Vector2f& uv, ColourValue colour);

		//! @brief Adds a face using a triangle fan topology. Fixed colour
		void AddFace(u32 vertexCount, const Vector3f* positions, ColourValue colour, bool flip = false);

		//! @brief Adds a face using a triangle fan topology with individual colours, normals and texture coordinates
		void AddFace(u32 vertexCount, const Vector3f* positions, const ColourValue* colours, const Vector3f* normals,
					 const Vector2f* uvs, bool flip = false);

		//! @brief  Adds a triangle connecting 3 vertices.
		void AddTriangle(u32 i0, u32 i1, u32 i2);

		//! @brief  Adds a flat triangle by adding vertices and indices.
		void AddTriangle(const Vector3f& P0, const Vector3f& P1, const Vector3f& P2, ColourValue colour,
						 bool flip = false);

		//! @brief  Adds a quad connecting 4 vertices.
		void AddQuad(u32 i0, u32 i1, u32 i2, u32 i3);

		//! @brief  Adds a flat quad by adding vertices and indices.
		void AddQuad(const Vector3f& P0, const Vector3f& P1, const Vector3f& P2, const Vector3f& P3, ColourValue colour,
					 bool flip = false);

		//! @brief  Builds a flat face given a set of set of positions.
		//! Planar UVs and Face Normals are calculated.
		void AddFlatFace(u32 vertexCount, const Vector3f* positions, ColourValue colour, bool flip = false);

		//! @brief Builds a flat face given a set of indices into a set of positions.
		//! This version helps when constructing flat faces from a set of vertex positions.
		//! Planar UVs and Face Normals are calculated.
		void AddIndexedFlatFace(u32 indexCount, const u32* indices, const Vector3f* positions, ColourValue colour,
								bool flip = false);

		//! @brief Extrudes a shape by duplicating the input shape and joining with quads.
		void Extrude(u32 vertexCount, const Vector3f* positions, ColourValue colour, f32 fLength);

		//! @brief Parses a string containing .obj formated mesh data.
		//! Adds the geometry into the current mesh.
		//! @param objString : the .obj formated string.
		//! @param colour : sets the uniform vertex colour.
		//! @param fScale : sets a uniform scaling factor applied to all positions.
		//! @return success or fail codes.
		result_t ParseObjFormat(std::string_view objString, ColourValue colour, f32 fScale);

		//! @brief Generates a set of tangents for the mesh in question.
		void GenerateTangents();

		//! @brief Finalizes the data and creates the mesh resource.
		//! returns the created mesh id.
		MeshId CreateMesh();

		//! @brief Resets the builder state.
		void Reset();

	private:
		std::vector<Vector3f> m_positions;
		std::vector<Vector3f> m_normals;
		std::vector<Vector2f> m_uvs;
		std::vector<ColourValue> m_colours;
		std::vector<u32> m_indices;
		std::vector<Vector4f> m_tangents;
	};
}

//------------------------------------------- Graphics/GraphicsApi.h -------------------------------------------

//--------------------------------------------- Graphics/Shader.h ---------------------------------------------

namespace Play3d::Graphics
{
	class Shader;

	using ShaderId = IdKey<Shader>;

	enum class ShaderType : u32
	{
		VERTEX_SHADER,
		PIXEL_SHADER,
		COMPUTE_SHADER,

		MAX_SHADER_TYPES,
		INVALID_SHADER_TYPE = ~0u
	};

	namespace ShaderStageFlag
	{
		using Type = u32;
		enum Enum : Type
		{
			VERTEX_STAGE = 1 << 0,
			PIXEL_STAGE = 1 << 1,
			COMPUTE_STAGE = 1 << 2,
		};
	}

	//! Construction from bytecode.
	struct ShaderDesc
	{
		ShaderType m_type = ShaderType::INVALID_SHADER_TYPE;
		std::string m_name;
		void* m_pByteCode = nullptr;
		size_t m_sizeBytes = 0;
	};

	namespace ShaderCompilationFlags
	{
		using Type = u32;
		enum Enum
		{
			DEBUG = 0x1,
			SOURCE_FILE = 0x2,
			DISABLE_STANDARD_MACROS = 0x4,
			ADD_RUNTIME_COMPILE_MACRO = 0x8
		};
	}

	//! Construction using the shader compiler.
	struct ShaderCompilerDesc
	{
		ShaderType m_type = ShaderType::INVALID_SHADER_TYPE;
		std::string m_name;
		std::string m_hlslCode;
		std::string m_entryPoint;
		std::vector<D3D_SHADER_MACRO> m_defines;
		u32 m_flags = 0;
	};

	class Shader
	{
	public:
		Shader(const ShaderDesc& rDesc);
		~Shader();

		ShaderType GetType() const { return m_shaderType; }

		void Bind(ID3D11DeviceContext* pContext);

		const void* GetByteCode() const { return m_pByteCode; }
		size_t GetByteCodeSize() const { return m_sizeBytes; }

		//! Compiles a shader from HLSL and adds it to the resource manager for shaders.
		static ShaderId Compile(const ShaderCompilerDesc& rDesc);
		static ShaderId LoadCompiledShaderFromFile(const char* pFilePath, ShaderType type);

	private:
		ShaderType m_shaderType;
		ComPtr<ID3D11DeviceChild> m_pShader;
		void* m_pByteCode;
		size_t m_sizeBytes;
	};
}

//-------------------------------------------- Graphics/Material.h --------------------------------------------

//--------------------------------------------- Graphics/Texture.h ---------------------------------------------

namespace Play3d::Graphics
{
	class Texture;
	using TextureId = IdKey<Texture>;

	enum class TextureFormat
	{
		GRAYSCALE,
		RGBA,
		BGRA,
		RGBA_F16,
		DEPTH,
	};

	namespace TextureFlags
	{
		using Type = u32;
		enum Bits
		{
			GENERATE_MIPS = 0x1,
			ENABLE_TEXTURE = 0x2,
			ENABLE_RENDER_TARGET = 0x4,
			ENABLE_DYNAMIC = 0x8,
		};
	}

	struct TextureDesc
	{
		u32 m_width = 0;
		u32 m_height = 0;
		TextureFormat m_format = TextureFormat::RGBA;
		const void* m_pImageData = nullptr;
		TextureFlags::Type m_flags = TextureFlags::GENERATE_MIPS | TextureFlags::ENABLE_TEXTURE;
		const char* m_pDebugName = nullptr;
	};

	struct TextureArrayDesc
	{
		u32 m_width = 0;
		u32 m_height = 0;
		TextureFormat m_format = TextureFormat::RGBA;
		std::vector<std::vector<u8>> m_ppImageData;
		TextureFlags::Type m_flags = TextureFlags::GENERATE_MIPS | TextureFlags::ENABLE_TEXTURE;
		const char* m_pDebugName = nullptr;
	};

	class Texture
	{
	public:
		Texture(const TextureDesc& rDesc);
		Texture(const TextureArrayDesc& rDesc);
		ID3D11Texture2D* GetTexture() { return m_pTexture.Get(); }
	private:
		friend class Graphics_Impl;
		ComPtr<ID3D11Texture2D> m_pTexture;
		ComPtr<ID3D11ShaderResourceView> m_pSRV;
		ComPtr<ID3D11RenderTargetView> m_pRTV;
		ComPtr<ID3D11DepthStencilView> m_pDSV;
	};
}

//--------------------------------------------- Graphics/Sampler.h ---------------------------------------------

namespace Play3d::Graphics
{
	class Sampler;
	using SamplerId = IdKey<Sampler>;

	enum class FilterMode
	{
		POINT,
		BILINEAR,
		ANISOTROPIC,
	};

	enum class AddressMode
	{
		CLAMP,
		WRAP,
		MIRROR,
		MIRROR_ONCE,
		BORDER
	};

	struct SamplerDesc
	{
		FilterMode m_filter = FilterMode::BILINEAR;
		AddressMode m_addressModeU = AddressMode::WRAP;
		AddressMode m_addressModeV = AddressMode::WRAP;
		AddressMode m_addressModeW = AddressMode::WRAP;
		ColourValue m_borderColour = Colour::Black;

		void SetUniformAddressMode(AddressMode mode)
		{
			m_addressModeU = m_addressModeV = m_addressModeW = mode;
		}
	};

	class Sampler
	{
	public:
		Sampler(const SamplerDesc& rDesc);

	private:
		friend class Graphics_Impl;
		ComPtr<ID3D11SamplerState> m_pSampler;
	};
}

namespace Play3d::Graphics
{
	class Material;
	using MaterialId = IdKey<Material>;

	//! @brief Rasterizer polygon fill mode setting used in Materials
	enum class FillMode
	{
		SOLID,
		WIREFRAME,
	};

	//! @brief Rasterizer polygon cull mode used in Materials
	enum class CullMode
	{
		BACK,
		FRONT,
		NONE
	};

	//! @brief Comparison functions used in several areas of the pipeline.
	enum class ComparisonFunc
	{
		NEVER,
		LESS,
		LESSEQUAL,
		EQUAL,
		GREATEREQUAL,
		GREATER,
		ALWAYS
	};

	//! @brief Output merger blend setting used in Materials
	enum class BlendMode
	{
		ADDITIVE,
		ALPHABLEND,
		MULTIPLY,
		NONE
	};

	//! @brief Constant determines the available texture slots that can be bound to a material.
	constexpr u32 kMaxMaterialTextureSlots = 4;

	//! @brief Constant determines first texture slots available for global texture bindings.
	constexpr u32 kGlobalTextureSlotStart = kMaxMaterialTextureSlots;

	//! @brief Constant determines first buffer slots available for global buffer bindings.
	constexpr u32 kGlobalBufferSlotStart = 4;

	//! @brief Pipeline state settings for the material.
	struct MaterialStateSettings
	{
		FillMode m_fillMode = FillMode::SOLID;
		CullMode m_cullMode = CullMode::BACK;
		BlendMode m_blendMode = BlendMode::NONE;
		ComparisonFunc m_depthComparison = ComparisonFunc::LESS;
		bool m_depthEnable = true;
		bool m_depthWrite = true;
		bool m_blendEnable = false;
	};

	//! @brief Constant data for common shaders and simpler materials.
	struct MaterialConstantData
	{
		Vector4f diffuseColour;
		Vector4f specularColour;
	};

	//! @brief For defining simple materials.
	//! Simple Materials use an internal shader that provides limited Texturing and Lighting.
	struct SimpleMaterialDesc
	{
		MaterialStateSettings m_state;
		MaterialConstantData m_constants;
		TextureId m_texture[kMaxMaterialTextureSlots];
		SamplerId m_sampler[kMaxMaterialTextureSlots];

		u32 m_lightCount = 1;
		bool m_bEnableLighting = false;
		bool m_bNullPixelShader = false;
	};

	//! @brief For defining custom materials with custom shaders and data.
	//! Complex Materials combine VS and PS shaders with a set of textures and some constant buffer data.
	//! Lighting data is available for you to work with in your own shaders.
	struct ComplexMaterialDesc
	{
		MaterialStateSettings m_state;
		ShaderId m_VertexShader;
		ShaderId m_PixelShader;
		TextureId m_texture[kMaxMaterialTextureSlots];
		SamplerId m_sampler[kMaxMaterialTextureSlots];
		const void* m_pConstantData = nullptr;
		size_t m_dataSize = 0;
		bool m_bNullPixelShader = false;

		//! @brief Helper method for setting up a material and compiling shaders.
		//! Also provides an example for setting up a ComplexMaterialDesc.
		ComplexMaterialDesc& SetupFromHLSLFile(const char* name, const char* hlslPath);

		//! @brief Helper method for setting up a material from compiled fxo shaders
		ComplexMaterialDesc& SetupFromCompiledShaders(const char* name, const char* fxoFolderPath);

		//! @brief Helper method for setting up a material and using compiled fxo shaders, then raw HLSL in that order.
		ComplexMaterialDesc& SetupFromEitherHLSLorCompiledShaders(const char* name, const char* path);
	};

	//! @brief Materials are resources that collect pipeline state settings and resources to bind on the pipeline.
	class Material
	{
	public:
		Material(const SimpleMaterialDesc& rDesc);
		Material(const ComplexMaterialDesc& rDesc);
		~Material();
		void SetTexture(u32 slot, TextureId id);
		void SetModified(bool bModified = true) { m_bModifiedFlag = bModified; }

	private:
		void SetupState(ID3D11Device* pDevice, const MaterialStateSettings& state);
		void SetupConstantBuffer(ID3D11Device* pDevice, const void* pData, size_t size);
		void SetupTextureBindings(ID3D11Device* pDevice, const TextureId* pTextureId, const SamplerId* pSamplerId);

	private:
		friend class Graphics_Impl;
		ComPtr<ID3D11RasterizerState> m_pRasterState;
		ComPtr<ID3D11DepthStencilState> m_pDepthStencilState;
		ComPtr<ID3D11BlendState> m_pBlendState;
		ComPtr<ID3D11Buffer> m_pMaterialConstants;
		ShaderId m_VertexShader;
		ShaderId m_PixelShader;
		TextureId m_texture[kMaxMaterialTextureSlots];
		SamplerId m_sampler[kMaxMaterialTextureSlots];
		bool m_bNullPixelShader = false;
		bool m_bModifiedFlag = false;
	};
}

//--------------------------------------------- Graphics/Buffer.h ---------------------------------------------

namespace Play3d::Graphics
{
	class Buffer;
	using BufferId = IdKey<Buffer>;

	//! Flags controlling the binding points in the pipeline that a buffer can be used.
	namespace BufferBindFlags
	{
		using Type = u32;
		enum Enum : u32
		{
			CONSTANT = 0x1u,
			VERTEX = 0x2u,
			INDEX = 0x4u,
			SRV = 0x08u,
			UAV = 0x10u,
		};
		static constexpr u32 kMaxBits = 5;
	};

	//! Additional buffer flags.
	namespace BufferFlags
	{
		using Type = u32;
		enum Enum : u32
		{
			DYNAMIC = 0x1u,
			IMMUTABLE = 0x2u,
			STRUCTURED = 0x4u,
			INDIRECT_ARGS = 0x8u
		};
		static constexpr u32 kMaxBits = 3;
	};

	struct BufferDesc
	{
		size_t m_sizeBytes = 0;
		const void* m_pInitialData = nullptr;
		const char* m_pDebugName = nullptr;
		BufferBindFlags::Type m_bindFlags = 0;
		BufferFlags::Type m_flags = 0;
		u32 m_structureStrideBytes = 0;

		BufferDesc()
			: m_sizeBytes(0)
			, m_pInitialData(nullptr)
			, m_pDebugName(nullptr)
			, m_bindFlags(0)
			, m_flags(0)
			, m_structureStrideBytes(0)
		{}

		template <typename T>
		BufferDesc& SetConstantBuffer(const T* pData)
		{
			m_sizeBytes = sizeof(T);
			m_pInitialData = pData;
			m_bindFlags = BufferBindFlags::CONSTANT;
			m_flags = BufferFlags::DYNAMIC;
			return *this;
		}

		template <typename T>
		BufferDesc& SetVertexBuffer(u32 vertexCount, const T* pVertices = nullptr)
		{
			m_sizeBytes = sizeof(T) * vertexCount;
			m_pInitialData = pVertices;
			m_bindFlags = BufferBindFlags::VERTEX;
			return *this;
		}

		template <typename T>
		BufferDesc& SetDynamicStructuredBuffer(u32 itemCount, const T* pItems = nullptr)
		{
			m_sizeBytes = sizeof(T) * itemCount;
			m_pInitialData = pItems;
			m_bindFlags = BufferBindFlags::SRV;
			m_flags = Graphics::BufferFlags::DYNAMIC | Graphics::BufferFlags::STRUCTURED;
			m_structureStrideBytes = sizeof(T);
			return *this;
		}

		template <typename T>
		BufferDesc& SetStructuredBuffer(u32 itemCount, const T* pItems = nullptr)
		{
			m_sizeBytes = sizeof(T) * itemCount;
			m_pInitialData = pItems;
			m_bindFlags = BufferBindFlags::SRV | BufferBindFlags::UAV;
			m_flags = BufferFlags::STRUCTURED;
			m_structureStrideBytes = sizeof(T);
			return *this;
		}

		BufferDesc& SetInitialData(const void* pData, size_t sizeBytes)
		{
			m_sizeBytes = sizeBytes;
			m_pInitialData = pData;
			return *this;
		}
	};

	class Buffer
	{
	public:
		Buffer(const BufferDesc& rDesc);

		ID3D11Buffer* GetBuffer();
		ID3D11ShaderResourceView* GetSRV();
		ID3D11UnorderedAccessView* GetUAV();

	private:
		ComPtr<ID3D11Buffer> m_pBuffer;
		ComPtr<ID3D11ShaderResourceView> m_pSRV;
		ComPtr<ID3D11UnorderedAccessView> m_pUAV;
	};
}

namespace Play3d::Graphics
{
	struct SurfaceSize
	{
		u32 m_width;
		u32 m_height;
	};

	//! @brief Obtains a transient (temporary) surface for rendering commands.
	//! This is useful for quickly rendering to an off-screen surface.
	//! A surface generated this way is only expected to exist until the frame has finished.
	//! Note: You must not hold onto these surface id's for longer than the current frame.
	TextureId GetTransientSurface(SurfaceSize surfaceSize, TextureFormat format);

	//! @brief Returns the size of the back-buffer surface.
	SurfaceSize GetDisplaySurfaceSize();

	//! @brief Sets the rendering outputs
	//! Passing a invalid texture id will set empty target slots
	void SetRenderTargets(TextureId colourTargetId, TextureId depthTextureId);

	//! @brief Sets multiple rendering outputs
	//! Passing a invalid texture ids will set empty target slots
	void SetRenderTargets(const TextureId* colourTargetIdArray, u32 mrtCount, TextureId depthTextureId);

	//! @brief Sets the output to use the swap chain surface.
	//! The default depth buffer is optional.
	void SetRenderTargetsToSwapChain(bool bEnableDefaultDepth);

	//! @brief Sets the output to use the swap chain surface with a custom depth buffer.
	void SetRenderTargetsToSwapChain(TextureId depthTextureId);

	//! @brief Clears the specified depth target.
	void ClearDepthTarget(TextureId textureId, f32 depthValue = 1.0f);

	//! @brief Clears specified target to a colour.
	void ClearRenderTarget(TextureId textureId, ColourValue clearColour);

	//! @brief Sets the default render target clear colour.
	void SetDefaultRenderTargetClearColour(ColourValue colour);

	//! @brief Sets the application into full screen or windowed.
	void SetFullscreenMode(bool bEnable);

	//! ---------------------------------------------------------
	//! Camera Viewport Interface
	//! ---------------------------------------------------------

	//! @brief A pipeline viewport structure.
	//! Note : Internally this should be binary compatible with D3D11_VIEWPORT
	struct Viewport
	{
		Viewport(){};
		Viewport(const SurfaceSize& s)
			: x(0.f)
			, y(0.f)
			, width((f32)s.m_width)
			, height((f32)s.m_height)
			, minDepth(0.f)
			, maxDepth(1.f){};

		f32 x;
		f32 y;
		f32 width;
		f32 height;
		f32 minDepth;
		f32 maxDepth;
	};

	//! @brief Sets the pipeline viewport transform.
	void SetViewport(const Viewport& viewport);

	//! @brief Sets the view matrix for future draw calls.
	//! Triggers generation of matrices in the per-frame constant buffer (prior to the next draw call).
	void SetViewMatrix(const Matrix4x4f& m);

	//! @brief Sets the projection matrix for future draw calls.
	//! Triggers generation of matrices in the per-frame constant buffer (prior to the next draw call).
	void SetProjectionMatrix(const Matrix4x4f& m);

	//! ---------------------------------------------------------
	//! Primitive Drawing Interface:
	//! Suitable for debugging, simple games or examples.
	//! These functions gather points, lines and triangles
	//! ---------------------------------------------------------

	//! @brief Begins the next batch for the debug drawing system
	//! Should be matched with a call to EndPrimitiveBatch()
	void BeginPrimitiveBatch();

	//! @brief Adds a single point to the current primitive batch.
	void DrawPoint(const Vector3f& v1, ColourValue colour);

	//! @brief Adds a single line to the current primitive batch with a uniform colour.
	void DrawLine(const Vector3f& v1, const Vector3f& v2, ColourValue colour);

	//! @brief Adds a single line to the current primitive batch with a colour transition.
	void DrawLine(const Vector3f& v1, const Vector3f& v2, ColourValue c1, ColourValue c2);

	//! @brief Adds a triangle to the current primitive batch with a uniform colour.
	void DrawTriangle(const Vector3f& v1, const Vector3f& v2, const Vector3f& v3, ColourValue colour);

	//! @brief Adds a triangle to the current primitive batch with a colour transition.
	void DrawTriangle(const Vector3f& v1, const Vector3f& v2, const Vector3f& v3, ColourValue c1, ColourValue c2,
					  ColourValue c3);

	//! @brief Adds a quad to the current primitive batch with a uniform colour.
 	void DrawQuad(const Vector3f& v1, const Vector3f& v2, const Vector3f& v3, const Vector3f& v4, ColourValue colour);

	//! @brief Adds a quad t o the current primitive batch with a colour transition.
	void DrawQuad(const Vector3f& v1, const Vector3f& v2, const Vector3f& v3, const Vector3f& v4, ColourValue c1,
				  ColourValue c2, ColourValue c3, ColourValue c4);

	//! @brief Ends the current batch in the debug drawing system.
	//! Draws all the lines in the batch.
	//! Closes BeginPrimitiveBatch()
	void EndPrimitiveBatch();

	//! ---------------------------------------------------------
	//! Mesh Drawing Interface:
	//! ---------------------------------------------------------

	//! @brief Draws an instance of a Mesh resource.
	//! Optionally specify a subrange of the mesh data.
	//! @param hMesh : the mesh resource to render.
	//! @param transform : a world transform to use when rendering.
	//! @param elementOffset : optional, offset into vertex or index buffers.
	//! @param elementCount : optional, size, default ~0u will render the entire mesh.
	void DrawMesh(MeshId hMesh, const Matrix4x4f& transform, u32 elementOffset = 0, u32 elementCount = ~0u);

	//! @brief Draws multiple instances of a Mesh resource using hardware instancing.
	//! Shaders must support the SV_INSTANCE semantic.
	void DrawInstancedMesh(MeshId hMesh, u32 kInstanceCount, u32 kInstanceOffset = 0, u32 elementOffset = 0, u32 elementCount = ~0u);

	//! @brief Makes a draw call without a vertex buffer.
	//! @param elements : Number of vertices to process 3 for a triangle.
	//! Useful for shader only effects / postfx
	//! Shaders must support the SV_VERTEX_ID semantic.
	void DrawWithoutVertices(u32 elements);

	struct IndirectDrawArgs
	{
		u32 vertexCount;
		u32 instanceCount;
		u32 vertexOffset;
		u32 instanceOffset;
	};

	//! @brief Performs an indirect draw call using a draw arguments buffer.
	//! The underlying DX11 draw call is : DrawInstancedIndirect()
	//! The buffer containing the draw arguments must conform to the IndirectDrawArgs structure.
	//! Buffer must have been created with BufferBindFlags::INDIRECT_ARGS
	void DrawIndirectWithoutVertices(BufferId hArgBuffer, u32 offset);

	//! ---------------------------------------------------------
	//! Materials Interface:
	//! ---------------------------------------------------------

	//! @brief Sets the current bound material
	void SetMaterial(MaterialId materialId);

	//! @brief Updates the shader constant buffer associated with this material.
	//! @param materialId : material concerned.
	//! @param pData : pointer to data block to be passed to a GPU constant buffer.
	//! @param size : size in bytes of the data block
	//! Take care with data alignment to match your shader constant buffer.
	void UpdateMaterialConstants(MaterialId materialId, const void* pData, size_t size);

	//! @brief Updates GPU texture data, data must be the original size.
	//! @param textureId : The texture we are updating.
	//! @param mipId : Index of mip level to update, 0 is largest
	//! @param sliceId : Index of texture slice to update, 0 is first
	//! @param pData : Image data, must be in the original pixel format specified when creating the texture, data is copied.
	//! @param rowPitchBytes : pitch in bytes of source image (e.g. width * 4 if RGBA)
	void UpdateTexture(TextureId textureId, u32 mipId, u32 sliceId, const void* pData, size_t rowPitchBytes);

	//! @brief Bind a texture to global slot.
	//! This is useful for environment maps or shadow maps where the texture is used for many draw calls.
	//! @param slot : register slot in the shader, use kMaxMaterialTextureSlots or greater.
	//! @param textureId : Texture to bind.
	//! @param samplerId : Sampler to bind.
	//! @param stageBinding : Shader stage visibility I.e. vertex and/or pixel.
	void BindGlobalTexture(u32 slot, TextureId textureId, SamplerId samplerId, ShaderStageFlag::Type stageBinding);

	//! ---------------------------------------------------------
	//! Buffer Interface:
	//! ---------------------------------------------------------

	//! @brief Updates GPU buffer data and allows binding to slots.
	void UpdateBuffer(BufferId bufferId, const void* pData, size_t size);

	//! @brief Binds a buffer global slot.
	//! @param bindFlags : determines the type of bind CONSTANT, SRV or UAV
	void BindGlobalBuffer(u32 slot, BufferId bufferId, ShaderStageFlag::Type stageBinding,
						  BufferBindFlags::Type bindFlags = BufferBindFlags::CONSTANT);

	//! ---------------------------------------------------------
	//! Lighting Interface
	//! ---------------------------------------------------------

	//! @brief Sets a light position for a given light index up to MaterialShaderKey::kMaxLights.
	void SetLightPosition(u32 index, const Vector3f& vPosition);

	//! @brief Sets a light direction for a given light index up to MaterialShaderKey::kMaxLights.
	void SetLightDirection(u32 index, const Vector3f& vDirection);

	//! @brief Sets a light colour for a given light index up to MaterialShaderKey::kMaxLights.
	void SetLightColour(u32 index, ColourValue colour);

	//! @brief Sets the ambient light colour.
	void SetAmbientLight(ColourValue colour);

	//! ---------------------------------------------------------
	//! Primitive Generation Interface
	//! ---------------------------------------------------------

	//! @brief Creates a Plane mesh.
	MeshId CreatePlane(f32 fWidth, f32 fHeight, ColourValue colour = Colour::White, f32 fUVScale = 1.0f);

	//! @brief Creates a Plane mesh in the XY plane with a lower left origin
	MeshId CreatePlaneXY(f32 fWidthX, f32 fHeightY, ColourValue colour = Colour::White, f32 fUVScale = 1.0f);

	//! @brief Creates a Cubic mesh with a given side length.
	MeshId CreateMeshCube(f32 size, ColourValue colour = Colour::White);

	//! @brief Creates a Box mesh with 3 side lengths.
	MeshId CreateMeshBox(f32 sizeX, f32 sizeY, f32 sizeZ, ColourValue colour = Colour::White);

	//! @brief Creates a Cylinder mesh for a given length and radius.
	//! @param segments : number of segments in the circular end section.
	MeshId CreateCylinder(f32 fLength, f32 fRadius, u32 segments, ColourValue colour = Colour::White);

	//! @brief Creates a Sphere mesh of given radius.
	//! @param segments : number of segments around the circumference.
	//! @param slices :  number of horizontal slices in the sphere.
	MeshId CreateSphere(f32 fRadius, u32 segments, u32 slices, ColourValue colour = Colour::White);

	//! ---------------------------------------------------------
	//! Platonic Solids
	//! (To fit inside Unit Sphere, as defined in Geometric Tools for Computer Graphics)
	//! ---------------------------------------------------------

	//! @brief Creates a Tetrahedron to fit in a unit sphere.
	MeshId CreatePlatonicTetrahedron(ColourValue colour = Colour::White);

	//! @brief Creates a Hexahedron to fit in a unit sphere.
	MeshId CreatePlatonicHexahedron(ColourValue colour = Colour::White);

	//! @brief Creates a Octahedron to fit in a unit sphere.
	MeshId CreatePlatonicOctahedron(ColourValue colour = Colour::White);

	//! @brief Creates a mesh by parsing an .obj formatted string.
	//! @param objString : string containing .obj formated data.
	//! @param colour : uniform colour applied to all vertices.
	//! @param fScale : uniform scaling factor applied to all positions.
	//! @return : the created mesh id.
	MeshId CreateMeshFromObjString(std::string_view objString, ColourValue colour = Colour::White, f32 fScale = 1.0f);

	//! @brief Creates a mesh by loading and parsing an .obj formatted file.
	//! @param filePath : path to the .obj file.
	//! @param colour : uniform colour applied to all vertices.
	//! @param fScale : uniform scaling factor applied to all positions.
	//! @return : the created mesh id.
	MeshId CreateMeshFromObjFile(const char* filePath, ColourValue colour = Colour::White, f32 fScale = 1.0f);

	//! ---------------------------------------------------------
	//! Textures
	//! ---------------------------------------------------------

	//! @brief Creates a Checkerboard texture with two colours.
	TextureId CreateTextureCheckerboard(u32 width, u32 height, ColourValue a, ColourValue b, u32 checkSize);

	//! @brief Creates a texture from a .png or .jpg file.
	TextureId CreateTextureFromFile(const char* pFilePath, bool bGenerateMipLevels = true);

	//! @brief Creates a Texture2dArray from series of equal sized .png or .jpg file.
	TextureId CreateTextureArrayFromFiles(std::initializer_list<const char*> pFilePaths);

	//! ---------------------------------------------------------
	//! Loading Image Data
	//! ---------------------------------------------------------

	//! @brief Loads image data into a byte array.
	//! Internally this uses a windows decoding framework supporting several formats .png and .jpg .
	//! @param path : path the the image file.
	//! @param imageOut : the byte array to load into, resized to fit.
	//! @param surfaceSizeOut : dimensions of the loaded image.
	//! @return : a success or fail code.
	result_t LoadImageAsRGBA(const char* path, std::vector<u8>& imageOut, SurfaceSize& surfaceSizeOut);

	//! ---------------------------------------------------------
	//! Texture Samplers
	//! ---------------------------------------------------------

	//! @brief Example creates a Linear mip-mapping sampler.
	//! Many other sampler options are available in the lower level interface.
	//! This function provides an example.
	//!
	SamplerId CreateLinearSampler();

	//! @brief Example creates a Point sampler.
	//! Many other sampler options are available in the lower level interface.
	//! This function provides an example.
	SamplerId CreatePointSampler();

	//! ---------------------------------------------------------
	//! Compute Shaders
	//! ---------------------------------------------------------

	//! @brief Binds a compute shader to the pipeline.
	//! @param shaderId : the compute shader id.
	void BindComputeShader(ShaderId shaderId);

	//! @brief Dispatches work to a compute pipeline.
	//! @param groupCountX : X dimension of thread groups to create.
	//! @param groupCountY : Y dimension of thread groups to create.
	//! @param groupCountZ : Z dimension of thread groups to create.
	//! Dispatches X*Y*Z thread groups in total.
	void Dispatch(u32 groupCountX, u32 groupCountY = 1, u32 groupCountZ = 1);

	class IGraphicsCallbacks
	{
	public:
		virtual void OnBeginFrame() = 0;
		virtual void OnEndFrame() = 0;
	};

	//! @brief returns the GPU frame time in milliseconds.
	float GetGPUFrameTime();

	//! @brief pushes a gpu marker into the GPU command buffer.
	void PushMarker(const char* name);

	//! @brief pops a gpu marker on the GPU command buffer.
	void PopMarker();

	//! @brief A callback signature that can be used to tap into the main window event routine.
	using WindowCallback = std::function<int(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)>;

	//! @brief Adds a windows message callback.
	void RegisterWindowCallback(WindowCallback callback);

	//! @brief Returns the underlying win32 handle to the main window.
	HWND GetWindowHandle();

	//! @brief returns the underlying DirectX11 Device object
	ID3D11Device* GetDevice();

	//! @brief returns the underlying DirectX11 Device Context object
	ID3D11DeviceContext* GetDeviceContext();

}

//------------------------------------------------- UI/UIApi.h -------------------------------------------------

//--------------------------------------------- Sprite/SpriteApi.h ---------------------------------------------

namespace Play3d::Sprite
{
	class SpriteAtlas;

	struct GridSpriteAtlasDesc
	{
		u32 imageWidth = 0;
		u32 imageHeight = 0;
		u32 gridSizeX = 1;
		u32 gridSizeY = 1;
	};

	struct UVRect
	{
		u32 x, y, width, height;
	};

	struct LooseSpriteAtlasDesc
	{
		u32 imageWidth = 0;
		u32 imageHeight = 0;
		u32 spriteCount = 0;
		const UVRect* uvArray = nullptr;
	};

	using SpriteAtlasId = IdKey<SpriteAtlas>;
	class SpriteAtlas
	{
	public:
		//! Describes the uvs of a sprite
		struct SpriteInfo
		{
			UVRect dimPixels;
			Vector4f uv[4];
			Vector4f offset;
		};

		//! Constructs a sprite atlas using a grid
		SpriteAtlas(const GridSpriteAtlasDesc& rDesc);

		//! Constructs a sprite atlas using loose collection of uv regions.
		//! Use this for packed sprite atlases.
		SpriteAtlas(const LooseSpriteAtlasDesc& rDesc);

		Graphics::BufferId GetBuffer() const { return m_bufferId; }
	private:
		void InternalBuild(const LooseSpriteAtlasDesc& rDesc);
		void UpdateGPUBuffer();
	private:
		u32 m_imageWidth = 0;
		u32 m_imageHeight = 0;
		std::vector<SpriteInfo> m_spriteInfo;
		Graphics::BufferId m_bufferId;
	};

	enum class SpriteBatchSortOrder
	{
		kNone,
		kFrontToBackZ,
		kBackToFrontZ,
	};

	//! @brief Returns matrix for transforming sprites on the screen.
	//! An orthographic matrix at 1 unit = 1 pixel with +Y down the screen.
	Matrix4x4f SpriteScreenMatrix();

	//! Begins a new batch of sprites from a specified atlas and texture specified in a material.
	//! Each batch can use a different texture and shader combination.
	//! Material Shaders must support sprite batch rendering.
	void BeginSpriteBatch(SpriteAtlasId hAtlas, Graphics::MaterialId materialId, const Matrix4x4f& viewProject,
						  SpriteBatchSortOrder sortOrder = SpriteBatchSortOrder::kNone);

	//! Appends a sprite to the current batch, nothing is drawn until the batch is closed with EndSpriteBatch().
	void DrawSprite(u32 spriteIndex, const Vector2f& vPosition, float fDepth, float fAngle, const Vector2f& vScale, const Vector2f& vOffset, ColourValue colour);

	//! Ends, closes the current sprite batch and dispatches rendering commands.
	void EndSpriteBatch();

	//! Creates a material based on an internal sprite shader using the specified texture.
	Graphics::MaterialId CreateSpriteMaterial(const char* pName, Graphics::TextureId hTexture,
											  bool bPointSample = false, bool bBlendEnable = true);

}

//------------------------------------------------- UI/Font.h -------------------------------------------------

namespace Play3d::UI
{
	class Font;
	using FontId = IdKey<Font>;

	struct FontDesc
	{
		std::string m_fontName; //! Windows font name
		std::string m_fontPath; //! Optional .ttf font path, the font is loaded for the duration of the program.
		std::string m_charSet;
		u32 m_pointSize;
		u32 m_textureWidth;
		u32 m_textureHeight;
	};

	struct FontVertex
	{
		Vector2f position;
		Vector2f uv;
		ColourValue colour;
	};

	class Font
	{
	public:
		Font(const FontDesc& rDesc);
		~Font();

		void DrawCharacterArray(ID3D11DeviceContext* pDC, const Vector2f& position, const char* pCharArray,
								const ColourValue* pColourArray, u32 lines,
								u32 columns);

		void DrawString(ID3D11DeviceContext* pDC, const Vector2f& position, ColourValue colour, std::string_view text);

	private:
		void Draw_Internal(ID3D11DeviceContext* pDC, u32 vertexCount);

		struct GlyphData
		{
			Vector2f uv0;
			Vector2f uv1;
			Vector2f offset0;
			Vector2f offset1;
			f32 xAdvance;
		};

		u8 m_glyphMap[256];
		std::vector<GlyphData> m_glyphData;
		ComPtr<ID3D11Texture2D> m_pTexture;
		ComPtr<ID3D11ShaderResourceView> m_pSRV;
		ComPtr<ID3D11RasterizerState> m_pRasterState;
		ComPtr<ID3D11SamplerState> m_pSampler;
		ComPtr<ID3D11Buffer> m_pVertexBuffer;
		u32 m_defaultAdvance;
	};

	class FontRenderSystem : public Graphics::IGraphicsCallbacks
	{
		PLAY_SINGLETON_INTERFACE(FontRenderSystem);

	public:
		FontRenderSystem();
		~FontRenderSystem();

		void OnBeginFrame() override;
		void OnEndFrame() override;

		void PrepFontDraw();
	private:
		ComPtr<ID3D11InputLayout> m_pFontInputLayout;

		ComPtr<ID3D11DepthStencilState> m_pDepthStencilState;
		ComPtr<ID3D11BlendState> m_pBlendState;

		Graphics::ShaderId m_fontVS;
		Graphics::ShaderId m_fontPS;
	};
}

namespace Play3d::UI
{
	FontId GetDebugFont();
	void DrawPrintf(FontId hFont, const Vector2f& position, ColourValue colour, const char* fmt, ...);
	void DrawString(FontId hFont, const Vector2f& position, ColourValue colour, std::string_view text);
	bool DrawButton(Sprite::SpriteAtlasId hAtlas, Graphics::TextureId texture, u32 index);
	void DrawCharacterArray(FontId hFont, const Vector2f& position, const char* pCharArray,
							const ColourValue* pColourArray, u32 lines, u32 columns);

}

//--------------------------------------------- System/SystemApi.h ---------------------------------------------
namespace Play3d
{
	//! @brief Defines global application settings.
	struct SystemDesc
	{
		const char* title = "Play3d - Default";
		u32 width = 800;
		u32 height = 600;
	};

	namespace System
	{
		//! @brief Returns true if the Play3d framework has been initialised.
		bool IsInitialised();

		//! @brief Used to initialise the Play3d framework.
		//! Call this once before using any other parts of the library.
		result_t Initialise(const SystemDesc& rDesc = SystemDesc());

		//! @brief Used to signal the beginning of a frame.
		//! Call this every frame.
		result_t BeginFrame();

		//! @brief Used to signal the end of a frame.
		//! Call this every frame.
		result_t EndFrame();

		//! @brief Used to shutdown the Play3d framework.
		//! Call this once when exiting your program.
		result_t Shutdown();

		//! @brief Returns the overall elapsed time since application start in seconds.
		//! Uses a high precision clock internally.
		f64 GetElapsedTime();

		//! @brief Loads in a PackFile for Play3D
		void AddPackFile(const char* filePath);

		//! @brief Returns the elapsed time since the previous frame in seconds.
		//! Uses a high precision clock internally.
		f32 GetDeltaTime();

		//! @brief Checks if a file exists with the specified path in both packed files or on disk.
		bool CheckFileExists(const char* filePath);

		//! @brief Checks if a file exists with the specified path on disk.
		bool CheckFileExistsOnDisk(const char* filePath);

		//! @brief Loads an entire file into memory returning it's size and a pointer loaded data.
		//! @param filePath : Path to the file.
		//! @param sizeOut : returns the size in bytes of the loaded data.
		//! @return a pointer to the loaded data. Use ReleaseFileData to free this memory.
		const void* LoadFileData(const char* filePath, size_t& sizeOut);

		//! @brief Releases memory allocated by LoadFileData .
		//! @param pMemory : a pointer returned by LoadFileData.
		void ReleaseFileData(const void* pMemory);
	}
}

//------------------------------------------ System/ServiceLocator.h ------------------------------------------

namespace Play3d
{
	template <typename T>
	class ServiceLocator
	{
		ServiceLocator() = delete;
		~ServiceLocator() = delete;

	public:
		template <typename... Args>
		static T& Initialise(Args... args)
		{
			if (!s_pService)
			{
				s_pService = new T(std::forward<Args>(args)...);
				PLAY_ASSERT(s_pService);
			}
			return *s_pService;
		}

		static void Shutdown()
		{
			if (s_pService)
			{
				delete s_pService;
				s_pService = nullptr;
			}
		}

		static T& Get()
		{
			PLAY_ASSERT(s_pService);
			return *s_pService;
		}

		static T* GetPtr() { return s_pService; }

		static bool IsValid() { return s_pService != nullptr; }

		static bool IsInvalid() { return !IsValid(); }

	private:
		inline static T* s_pService = nullptr;
	};
}

#define PLAY_SERVICE_LOCATOR_ONLY(classname)                                                                           \
	PLAY_NONCOPYABLE(classname)                                                                                        \
	friend class Play3d::ServiceLocator<classname>;
//--------------------------------------------- System/Packfile.h ---------------------------------------------

namespace Play3d::Packfile
{

	struct PackedFileHeader
	{
		size_t dataBlockOffset;
		size_t dataBlockSize;
		size_t dirBlockOffset;
		size_t dirEntryCount;
	};

	struct PackedFileEntry
	{
		static constexpr size_t PLAY_MAX_PATH_LENGTH = 256;
		char path[PLAY_MAX_PATH_LENGTH];
		size_t offset;
		size_t dataSize;
	};

	class PackedFile
	{
	public:
		PackedFile(const void* pData, size_t sizeBytes);

		~PackedFile();

		const void* GetFile(const char* pFilePath, size_t& sizeBytes);

		const size_t GetNumberOfFilesInPackFile();

		const PackedFileEntry* GetFileInfoAtIndex(size_t index);

		bool CheckPointerIsInPack(const void* ptr);

	private:
		const void* m_pRawData = nullptr;
		const PackedFileHeader* m_pHeader = nullptr;
		const PackedFileEntry* m_pDirBlock = nullptr;
		const uint8_t* m_pDataBlock = nullptr;
		size_t m_totalSizeBytes = 0;
	};

	class PackedFileManager
	{
		PLAY_NONCOPYABLE(PackedFileManager);

		PackedFileManager() = default;
		~PackedFileManager();

	public:
		static PackedFileManager& Instance() { return *ms_pInstance; }
		static void Initialise();
		static void Destroy();
		void AddPackFile(const char* pFilePath);
		const void* FindFile(const char* pFilePath, size_t& sizeBytes);
		bool CheckPointerIsInPack(const void* ptr);

	private:
		static PackedFileManager* ms_pInstance;

		std::vector<PackedFile*> m_packedFiles;
	};

}

//----------------------------------------------- Demo/DemoApi.h -----------------------------------------------

namespace Play3d::Demo
{
	//! ---------------------------------------------------------
	//! Demo Features for Tutorials

	//! @brief Sets the position and view angle for the demo camera.
	//! @param position : the eye position.
	//! @param fAzimuth : the view angle in the horizontal plane.
	//! @param fElevation : the view angle in the vertical plane.
	void SetDebugCameraPosition(const Vector3f& position, f32 fAzimuth = 0.f, f32 fElevation = 0.f);

	//! @brief Sets the field of view and Z clipping for the demo camera.
	//! @param fFOV : the vertical field of view angle in radians.
	//! @param fNearClip : the near Z clipping plane.
	//! @param fFarClip : the far Z clipping plane.
	void SetDebugCameraFOV(f32 fFOV, f32 fNearClip = 0.1f, f32 fFarClip = 100.f);

	//! @brief Updates the view and projection matrices for the demo camera.
	void UpdateDebugCamera();

	//! @brief Applies the demo camera matrices.
	//! Passes them to the GPU for draw routines.
	void SetDebugCameraMatrices();

	//! @brief Draws a grid in the XZ plane and RGB axis marker at the origin.
	//! @param fInterval : Grid regularity.
	//! @param fAxisLength : Length of axis marker.
	void DrawDebugGrid(f32 fInterval = 1.0f, f32 fAxisLength = 1.0f);

	//! @brief Example material constants.
	//! We can provide extra shader constant data to a material.
	//! This data is visible in a custom shader constant buffer.
	struct CustomMaterialConstants
	{
		Matrix4x4f lightMatrix;
	};

	//! @brief Creates an example material with a custom shader.
	//! @return : A new material id.
	Graphics::MaterialId CreateComplexMaterial();
}

#endif // __PLAY3D__

#ifdef PLAY_IMPLEMENTATION
////////////////// BEGIN IMPLEMENTATION SECTION //////////////////////////
#include <cstdarg>
#include <cstdio>
#include <windows.h>
#include <format>
#include <charconv>
#include <d3dcompiler.h>
#include <Xinput.h>
#include <cstdint>
#include <cstring>
//--------------------------------------------------------------------------------------------------------------

//--------------------------------------------- Audio/AudioApi.cpp ---------------------------------------------

//--------------------------------------------- Audio/Audio_Impl.h ---------------------------------------------

namespace Play3d::Audio
{
	class Audio_Impl
	{
		PLAY_NONCOPYABLE(Audio_Impl);
		PLAY_SINGLETON_INTERFACE(Audio_Impl);

		Audio_Impl();
		~Audio_Impl();

	public:
		void BeginFrame();

		void EndFrame();

		VoiceId PlaySound(SoundId soundId, f32 fGain, f32 fPan);

		void StopSound(VoiceId voiceId);

		IXAudio2* GetDevice();

	private:
		u32 AllocateVoice();
		void ReleaseVoice(u32 slot);

		class AudioVoiceCB : public IXAudio2VoiceCallback
		{
		public:
			AudioVoiceCB()
				: m_bFinished(false)
			{}
			~AudioVoiceCB() {}
			void OnStreamEnd() { AllowRelease(); }
			void OnVoiceProcessingPassEnd() {}
			void OnVoiceProcessingPassStart(UINT32 SamplesRequired) {}
			void OnBufferEnd(void* pBufferContext) {}
			void OnBufferStart(void* pBufferContext) {}
			void OnLoopEnd(void* pBufferContext) {}
			void OnVoiceError(void* pBufferContext, HRESULT Error) { AllowRelease(); }

			bool IsFinished() const { return m_bFinished; }
			void Reset() { m_bFinished = false; }
			void AllowRelease() { m_bFinished = true; }

		private:
			std::atomic<bool> m_bFinished;
		};

		struct InternalVoice
		{
			IXAudio2SourceVoice* m_pVoice;
			AudioVoiceCB* m_pCallback;
		};

	private:
		ComPtr<IXAudio2> m_pXAudio2;
		IXAudio2MasteringVoice* m_pMasterVoice;
		std::vector<InternalVoice> m_voices;
		std::vector<u32> m_freelist;
	};
}

namespace Play3d::Audio
{
	SoundId LoadSoundFromFile(const char* filePath, bool bEnableLooping)
	{
		SoundId soundId;
		size_t sizeBytes;
		const void* pData = System::LoadFileData(filePath, sizeBytes);
		if (pData)
		{
			SoundDesc desc;
			desc.m_pData = pData;
			desc.m_sizeBytes = sizeBytes;
			desc.m_bLoop = bEnableLooping;
			soundId = Resources::CreateAsset<Sound>(desc);
		}
		return soundId;
	}

	VoiceId PlaySound(SoundId soundId, f32 fGain, f32 fPan)
	{
		return Audio_Impl::Instance().PlaySound(soundId, fGain, fPan);
	}

	void StopSound(VoiceId voiceId)
	{
		return Audio_Impl::Instance().StopSound(voiceId);
	}

	void* GetAudioDevice()
	{
		return Audio_Impl::Instance().GetDevice();
	}

}

//-------------------------------------------- Audio/Audio_Impl.cpp --------------------------------------------

#pragma comment(lib, "xaudio2.lib")

namespace Play3d::Audio
{
	PLAY_SINGLETON_IMPL(Audio_Impl);

	Audio_Impl::Audio_Impl()
		: m_pMasterVoice(nullptr)
	{
		HRESULT hr;
		hr = XAudio2Create(&m_pXAudio2, 0, XAUDIO2_DEFAULT_PROCESSOR);
		PLAY_ASSERT_MSG(SUCCEEDED(hr), "Failed XAudio create.");

#ifdef _DEBUG
		XAUDIO2_DEBUG_CONFIGURATION debugConfig = {};
		debugConfig.TraceMask = XAUDIO2_LOG_ERRORS;
		debugConfig.BreakMask = XAUDIO2_LOG_ERRORS;
		debugConfig.LogThreadID = TRUE;
		debugConfig.LogFileline = TRUE;
		debugConfig.LogFunctionName = TRUE;
		debugConfig.LogTiming = TRUE;
		m_pXAudio2->SetDebugConfiguration(&debugConfig, NULL);
#endif

		hr = m_pXAudio2->CreateMasteringVoice(&m_pMasterVoice);
		PLAY_ASSERT_MSG(SUCCEEDED(hr), "Failed XAudio create master voice.");
	}

	Audio_Impl::~Audio_Impl()
	{
		for (InternalVoice& it : m_voices)
		{
			if(it.m_pVoice)
				it.m_pVoice->DestroyVoice();
			PLAY_SAFE_DELETE(it.m_pCallback);
		}

		if (m_pMasterVoice)
		{
			m_pMasterVoice->DestroyVoice();
		}

		if (m_pXAudio2)
		{
			m_pXAudio2->StopEngine();
			Sleep(1000);
			m_pXAudio2.Reset();
		}
	}

	void Audio_Impl::BeginFrame()
	{}

	void Audio_Impl::EndFrame()
	{
		for (u32 slot = 0; slot < (u32)m_voices.size(); ++slot)
		{
			if (m_voices[slot].m_pCallback->IsFinished())
			{
				if (m_voices[slot].m_pVoice)
				{
					m_voices[slot].m_pVoice->DestroyVoice();
					m_voices[slot].m_pVoice = nullptr;
				}
				m_voices[slot].m_pCallback->Reset();

				ReleaseVoice(slot);
			}
		}
	}

	VoiceId Audio_Impl::PlaySound(SoundId soundId, f32 fGain, f32 fPan)
	{
		VoiceId voiceId;
		HRESULT hr;
		u32 slot = AllocateVoice();
		if (slot != ~0)
		{
			Sound* pSound = Resources::ResourceManager<Sound>::Instance().GetPtr(soundId);
			if (pSound)
			{
				IXAudio2SourceVoice* pVoice = nullptr;
				hr = m_pXAudio2->CreateSourceVoice(&pVoice,
												   &pSound->m_format,
												   0,
												   XAUDIO2_DEFAULT_FREQ_RATIO,
												   m_voices[slot].m_pCallback);
				PLAY_ASSERT(SUCCEEDED(hr));

				hr = pVoice->SubmitSourceBuffer(&pSound->m_buffer);
				PLAY_ASSERT(SUCCEEDED(hr));

				hr = pVoice->SetVolume(fGain);
				PLAY_ASSERT(SUCCEEDED(hr));

				f32 fLeft = cos(fPan * kfHalfPi);
				f32 fRight = sin(fPan * kfHalfPi);
				f32 channelVolumes[] = {fLeft, fRight};
				hr = pVoice->SetChannelVolumes(2, channelVolumes);
				PLAY_ASSERT(SUCCEEDED(hr));

				hr = pVoice->Start();
				PLAY_ASSERT(SUCCEEDED(hr));

				m_voices[slot].m_pVoice = pVoice;

				voiceId = VoiceId(slot);
			}
		}
		return voiceId;
	}

	void Audio_Impl::StopSound(VoiceId voiceId)
	{
		HRESULT hr;
		if (voiceId.IsValid())
		{
			u32 slot = voiceId.GetValue();
			hr = m_voices[slot].m_pVoice->Stop(0, 0);
			hr = m_voices[slot].m_pVoice->FlushSourceBuffers();
			m_voices[slot].m_pCallback->AllowRelease();
		}
	}

	u32 Audio_Impl::AllocateVoice()
	{
		u32 slot = ~0u;
		if (m_freelist.empty())
		{
			slot = (u32)m_voices.size();
			InternalVoice v;
			v.m_pCallback = new AudioVoiceCB();
			v.m_pVoice = nullptr;
			m_voices.push_back(v);
		}
		else
		{
			slot = m_freelist.back();
			m_freelist.pop_back();
		}
		return slot;
	}

	void Audio_Impl::ReleaseVoice(u32 slot)
	{
		m_freelist.push_back(slot);
	}

	IXAudio2* Audio_Impl::GetDevice()
	{
		return m_pXAudio2.Get();
	}

}

//---------------------------------------------- Audio/Sound.cpp ----------------------------------------------

namespace Play3d
{
	namespace Audio
	{
		struct RiffChunk
		{
			u32 m_id;
			u32 m_size;
		};

		Sound::Sound(const SoundDesc& rDesc)
			: m_format{}
			, m_buffer{}
			, m_pData(nullptr)
			, m_sizeBytes(0)
		{

			const u8* p(static_cast<const u8*>(rDesc.m_pData));
			const u8* pEnd(p + rDesc.m_sizeBytes);

			const RiffChunk* pRiffChunk(reinterpret_cast<const RiffChunk*>(p));
			PLAY_ASSERT_MSG(pRiffChunk->m_id == 'FFIR', "RIFF chunk invalid");
			p += sizeof(RiffChunk);

			const u32* pWave(reinterpret_cast<const u32*>(p));
			PLAY_ASSERT_MSG(*pWave == 'EVAW', "WAVE chunk invalid");
			p += sizeof(u32);

			const RiffChunk* pChunk(reinterpret_cast<const RiffChunk*>(p));
			p += sizeof(RiffChunk);

			bool bFmt = false;
			bool bData = false;

			while(p < pEnd)
			{
				switch (pChunk->m_id)
				{
				case ' tmf':
					PLAY_ASSERT_MSG(sizeof(PCMWAVEFORMAT) <= pChunk->m_size, "_fmt chunk invalid");
					memcpy(&m_format, p, sizeof(PCMWAVEFORMAT));
					bFmt = true;
					break;

				case 'atad':
					m_pData = _aligned_malloc(pChunk->m_size, 16);
					PLAY_ASSERT(m_pData);
					memcpy(m_pData, p, pChunk->m_size);
					m_sizeBytes = pChunk->m_size;
					bData = true;
					break;
				}

				p += pChunk->m_size;

				pChunk = reinterpret_cast<const RiffChunk*>(p);
				p += sizeof(RiffChunk);
			}

			PLAY_ASSERT_MSG(bFmt && bData, "Invalid Sound Data");

			m_buffer.AudioBytes = static_cast<u32>(m_sizeBytes);
			m_buffer.pAudioData = static_cast<const BYTE*>(m_pData);
			m_buffer.Flags = XAUDIO2_END_OF_STREAM;

			m_buffer.LoopBegin = 0u;
			m_buffer.LoopLength = 0u;
			m_buffer.LoopCount = rDesc.m_bLoop ? XAUDIO2_LOOP_INFINITE : 0;
		}

		Sound::~Sound()
		{
			if (m_pData)
			{
				_aligned_free(m_pData);
			}
		}

	}
}

//--------------------------------------------- Debug/TraceApi.cpp ---------------------------------------------

namespace Play3d::Debug
{
	constexpr u32 kMaxLogBuffer = 2048;

	void Put(const char* pStr)
	{
		::OutputDebugStringA(pStr);
		::fputs(pStr, stdout);
	}

	void Printf(const char* pFmtStr, ...)
	{
		char strBuffer[kMaxLogBuffer];
		va_list args;
		va_start(args, pFmtStr);
		vsprintf_s(strBuffer, kMaxLogBuffer, pFmtStr, args);
		va_end(args);

		Debug::Put(strBuffer);
	}

	void Tracef(const char* pStrFilename, unsigned int lineNum, const char* pFmtStr, ...)
	{
		char strBuffer[kMaxLogBuffer];
		va_list args;
		va_start(args, pFmtStr);
		vsprintf_s(strBuffer, kMaxLogBuffer, pFmtStr, args);
		va_end(args);

		Debug::Printf("%s(%u): %s\n", pStrFilename, lineNum, strBuffer);
	}

	void HLine()
	{
		Debug::Put("--------------------------------------------------------\n");
	}
}

//---------------------------------------------- Demo/DemoApi.cpp ----------------------------------------------

namespace Play3d::Demo
{
	struct DemoImpl
	{
		Vector3f m_eyePosition = Vector3f(0.f, 1.f, -3.5f);
		Vector3f m_vForward = Vector3f(0, 0, 1);
		Vector3f m_vRight = Vector3f(1, 0, 0);
		Vector3f m_vUp = Vector3f(0, 1, 0);

		f32 m_fAzimuth = 0.f;
		f32 m_fElevation = kfPi / 16;

		f32 m_fNearClip = 0.1f;
		f32 m_fFarClip = 100.f;

		f32 m_fFOV = kfPi / 4.f;

		bool m_bIsCaptured = false;
	};

	static DemoImpl s_demoState;

	void SetDebugCameraPosition(const Vector3f& fPosition, f32 fAzimuth, f32 fElevation)
	{
		s_demoState.m_eyePosition = fPosition;
		s_demoState.m_fAzimuth = fAzimuth;
		s_demoState.m_fElevation = fElevation;
	}

	void SetDebugCameraFOV(f32 fFOV, f32 fNearClip, f32 fFarClip)
	{
		s_demoState.m_fFOV = fFOV;
		s_demoState.m_fNearClip = fNearClip;
		s_demoState.m_fFarClip = fFarClip;
	}

	void UpdateDebugCamera()
	{
		constexpr f32 kfRotateSpeed = 0.005f;
		constexpr f32 kfSpeed = 1.0f;

		f32 dT = System::GetDeltaTime();
		Input::MouseState mouseState = Input::GetMouseState();

		if (mouseState.m_rightButton)
		{
			if (!s_demoState.m_bIsCaptured)
			{
				Input::CaptureMouse(true);
				s_demoState.m_bIsCaptured = true;
			}

			s_demoState.m_fAzimuth += mouseState.m_deltaX * -kfRotateSpeed;
			s_demoState.m_fElevation += mouseState.m_deltaY * kfRotateSpeed;

			if (s_demoState.m_fAzimuth > kfPi)
			{
				s_demoState.m_fAzimuth -= kfTwoPi;
			}
			if (s_demoState.m_fAzimuth < -kfPi)
			{
				s_demoState.m_fAzimuth += kfTwoPi;
			}

			if (s_demoState.m_fElevation > kfHalfPi)
			{
				s_demoState.m_fElevation = kfHalfPi;
			}
			if (s_demoState.m_fElevation < -kfHalfPi)
			{
				s_demoState.m_fElevation = -kfHalfPi;
			}
		}
		else if (s_demoState.m_bIsCaptured)
		{
			Input::CaptureMouse(false);
			s_demoState.m_bIsCaptured = false;
		}

		f32 cosAz = cos(s_demoState.m_fAzimuth);
		f32 sinAz = sin(s_demoState.m_fAzimuth);
		f32 cosEl = cos(s_demoState.m_fElevation);
		f32 sinEl = sin(s_demoState.m_fElevation);

		s_demoState.m_vRight = Vector3f(cosAz, 0.f, -sinAz);
		s_demoState.m_vUp = Vector3f(sinAz * sinEl, cosEl, cosAz * sinEl);
		s_demoState.m_vForward = Vector3f(sinAz * cosEl, -sinEl, cosAz * cosEl);

		Vector3f vDelta(0.f, 0.f, 0.f);

		if (Input::IsKeyDown('W'))
		{
			vDelta.z += 1;
		}
		if (Input::IsKeyDown('S'))
		{
			vDelta.z += -1;
		}
		if (Input::IsKeyDown('A'))
		{
			vDelta.x += 1;
		}
		if (Input::IsKeyDown('D'))
		{
			vDelta.x += -1;
		}
		if (Input::IsKeyDown('E'))
		{
			vDelta.y += -1;
		}
		if (Input::IsKeyDown('Q'))
		{
			vDelta.y += 1;
		}

		s_demoState.m_eyePosition += s_demoState.m_vForward * vDelta.z * dT * kfSpeed;
		s_demoState.m_eyePosition += s_demoState.m_vRight * vDelta.x * dT * kfSpeed;
		s_demoState.m_eyePosition += s_demoState.m_vUp * vDelta.y * dT * kfSpeed;
	}

	void SetDebugCameraMatrices()
	{
		Graphics::SurfaceSize surfaceSize = Graphics::GetDisplaySurfaceSize();

		f32 aspect((f32)surfaceSize.m_width / (f32)surfaceSize.m_height);

		Matrix4x4f view = MatrixLookatRH(s_demoState.m_eyePosition,
										 s_demoState.m_eyePosition + s_demoState.m_vForward,
										 s_demoState.m_vUp);
		Matrix4x4f project =
			MatrixPerspectiveProjectRH(s_demoState.m_fFOV, aspect, s_demoState.m_fNearClip, s_demoState.m_fFarClip);

		Graphics::SetViewport(Graphics::Viewport(surfaceSize));
		Graphics::SetViewMatrix(view);
		Graphics::SetProjectionMatrix(project);
	}

	void DrawDebugGrid(f32 fInterval, f32 fAxisLength)
	{
		Graphics::BeginPrimitiveBatch();

		Graphics::DrawLine(Vector3f(0, 0, 0), Vector3f(fAxisLength, 0, 0), Colour::Red);
		Graphics::DrawLine(Vector3f(0, 0, 0), Vector3f(0, fAxisLength, 0), Colour::Green);
		Graphics::DrawLine(Vector3f(0, 0, 0), Vector3f(0, 0, fAxisLength), Colour::Blue);

		const int kSize = 10;
		const f32 fSize = (f32)kSize;

		for (int i = -kSize; i <= kSize; ++i)
		{
			f32 t = fInterval * i;
			Graphics::DrawLine(Vector3f(t, 0, -fSize), Vector3f(t, 0, fSize), Colour::Darkgray);
			Graphics::DrawLine(Vector3f(-fSize, 0, t), Vector3f(fSize, 0, t), Colour::Darkgray);
		}

		Graphics::EndPrimitiveBatch();
	}

	Graphics::MaterialId CreateComplexMaterial()
	{
		using namespace Graphics;
		ShaderId customPixelShader;
		{
			static const char* hlslCode = R"(
				struct PSInput
				{
					float4 position : SV_POSITION;
					float4 colour : COLOUR;
				};
				float4 PS_Main(PSInput input) : SV_TARGET
				{
					return float4(input.position.x * 0.001, input.position.y*0.001, 0.5, 1.0);
				})";

			ShaderCompilerDesc compilerOptions = {};
			compilerOptions.m_name = "Example PS Shader";
			compilerOptions.m_type = Graphics::ShaderType::PIXEL_SHADER;
			compilerOptions.m_hlslCode = hlslCode;
			compilerOptions.m_entryPoint = "PS_Main";
			compilerOptions.m_defines.push_back({"LIGHTS", "4"});
			customPixelShader = Shader::Compile(compilerOptions);
		}

		CustomMaterialConstants data;
		data.lightMatrix = MatrixIdentity<4, f32>();

		ComplexMaterialDesc desc = {};
		desc.m_state.m_cullMode = Graphics::CullMode::BACK;
		desc.m_state.m_fillMode = Graphics::FillMode::SOLID;
		desc.m_PixelShader = customPixelShader;
		desc.m_pConstantData = &data;
		desc.m_dataSize = sizeof(CustomMaterialConstants);

		return Resources::CreateAsset<Graphics::Material>(desc);
	}
}

//-------------------------------------------- Graphics/Buffer.cpp --------------------------------------------

//------------------------------------------ Graphics/Graphics_Impl.h ------------------------------------------

//--------------------------------------- Graphics/PrimitiveBatch_Impl.h ---------------------------------------

namespace Play3d::Graphics
{
	struct PrimitiveVertex
	{
		Vector3f position;
		ColourValue colour;
	};

	//! @brief Collects individual points, lines and triangles into a batch of primitives
	class PrimitiveBatch
	{
	public:
		PrimitiveBatch(ID3D11Device* pDevice, u32 kMaxVertexCount);
		~PrimitiveBatch();

		void AppendPoint(const Vector3f& v1, ColourValue c1);
		void AppendLine(const Vector3f& v1, const Vector3f& v2, ColourValue c1, ColourValue c2);
		void AppendTriangle(const Vector3f& v1, const Vector3f& v2, const Vector3f& v3, ColourValue c1, ColourValue c2,
							ColourValue c3);

		void Flush(ID3D11DeviceContext* pContext);

		void Bind(ID3D11DeviceContext* pContext);

		void DrawPoints(ID3D11DeviceContext* pContext);

		void DrawLines(ID3D11DeviceContext* pContext);

		void DrawTriangles(ID3D11DeviceContext* pContext);

	private:
		std::vector<PrimitiveVertex> m_points;
		std::vector<PrimitiveVertex> m_lines;
		std::vector<PrimitiveVertex> m_triangles;

		ComPtr<ID3D11Buffer> m_pVertexBuffer;

		u32 m_totalVertexCount;
		u32 m_maxVertexCount;
		u32 m_pointVertexCount;
		u32 m_lineVertexCount;
		u32 m_triangleVertexCount;
	};

	//! @brief Manages the allocation and rendering of primitive batches.
	class PrimitiveRenderSystem : public IGraphicsCallbacks
	{
		PLAY_SINGLETON_INTERFACE(PrimitiveRenderSystem);
	public:
		PrimitiveRenderSystem();
		~PrimitiveRenderSystem();
		void OnBeginFrame() override;
		void OnEndFrame() override;

		PrimitiveBatch* AllocatePrimitiveBatch();
		void DrawPrimitiveBatch(PrimitiveBatch* pBatch);

	private:
		std::vector<PrimitiveBatch*> m_primitiveBatchRing;
		ComPtr<ID3D11InputLayout> m_pPrimitiveInputLayout;

		ComPtr<ID3D11RasterizerState> m_pRasterState;
		ComPtr<ID3D11DepthStencilState> m_pDepthStencilState;
		ComPtr<ID3D11BlendState> m_pBlendState;

		ShaderId m_primitiveBatchVS;
		ShaderId m_primitiveBatchPS;
		u32 m_nNextPrimitiveBatch;
	};

}

//------------------------------------ Graphics/ShaderConstantBuffer_Impl.h ------------------------------------

namespace Play3d::Graphics
{
	template <typename T>
	class ShaderConstantBuffer_Impl
	{
	public:
		ShaderConstantBuffer_Impl()
			: m_bIsDirty(false)
		{
			memset(&m_cpuData, 0, sizeof(T));
		}
		void Init(ID3D11Device* pDevice)
		{
			D3D11_BUFFER_DESC desc = {};
			desc.ByteWidth = sizeof(T);
			desc.Usage = D3D11_USAGE::D3D11_USAGE_DYNAMIC;
			desc.BindFlags = D3D11_BIND_FLAG::D3D11_BIND_CONSTANT_BUFFER;
			desc.CPUAccessFlags = D3D11_CPU_ACCESS_WRITE;
			desc.MiscFlags = 0;
			desc.StructureByteStride = 0;

			HRESULT hr = pDevice->CreateBuffer(&desc, NULL, &m_pBuffer);
			PLAY_ASSERT_MSG(SUCCEEDED(hr), "Could not create Constant Buffer");

			m_bIsDirty = true;
		}

		void Bind(ID3D11DeviceContext* pDC, u32 slot)
		{
			ID3D11Buffer* buffers[] = {m_pBuffer.Get()};
			pDC->VSSetConstantBuffers(slot, 1, buffers);
			pDC->PSSetConstantBuffers(slot, 1, buffers);
			pDC->CSSetConstantBuffers(slot, 1, buffers);
		}

		void UpdateGPU(ID3D11DeviceContext* pDC)
		{
			if (m_bIsDirty)
			{
				D3D11_MAPPED_SUBRESOURCE data;
				HRESULT hr = pDC->Map(m_pBuffer.Get(), 0, D3D11_MAP::D3D11_MAP_WRITE_DISCARD, 0, &data);
				if (SUCCEEDED(hr))
				{
					memcpy(data.pData, &m_cpuData, sizeof(T));
					pDC->Unmap(m_pBuffer.Get(), 0);
				}
				m_bIsDirty = false;
			}
		}

		bool IsDirty() const { return m_bIsDirty; }

		T& Get()
		{
			m_bIsDirty = true;
			return m_cpuData;
		}

		const T& Get() const { return m_cpuData; }

	private:
		T m_cpuData;
		ComPtr<ID3D11Buffer> m_pBuffer;
		bool m_bIsDirty = false;
	};
}

namespace Play3d
{
	struct SystemDesc;
}

namespace Play3d::Graphics
{
	struct MaterialShaderKey
	{
		union {
			struct
			{
				u32 m_pixelShader  : 1;
				u32 m_useTexture0  : 1;
				u32 m_useNormalMap : 1;
				u32 m_useLighting  : 1;
				u32 m_lightCount   : 2;
			} m_bits;
			u32 m_value;
		};

		static constexpr u32 kUsedBits = 6;
		static constexpr u32 kPermutations = 1 << kUsedBits;
		static constexpr u32 kMaxLights = 4;
	};

	class Graphics_Impl
	{
		PLAY_NONCOPYABLE(Graphics_Impl);

	public:
		static Graphics_Impl& Instance() { return *ms_pInstance; }
		static void Initialise(const SystemDesc& rDesc);
		static void Destroy();

		result_t PostInitialise();
		result_t BeginFrame();

		void ClearStateCache();

		result_t EndFrame();

		void Flush();

		HINSTANCE GetHInstance() const { return m_hInstance; }
		HWND GetHWnd() const { return m_hWnd; }

		ID3D11Device* GetDevice() const { return m_pDevice.Get(); }
		ID3D11DeviceContext* GetDeviceContext() const { return m_pDeviceContext.Get(); }

		void SetFullscreenMode(bool bEnable);

		void DrawMesh(const Mesh* pMesh, u32 elementOffset, u32 elementCount);

		void DrawInstancedMesh(const Mesh* pMesh, u32 kInstanceCount, u32 kInstanceOffset, u32 elementOffset,
							   u32 elementCount);

		void DrawWithoutVertices(u32 elements);

		void DrawIndirectWithoutVertices(BufferId hArgBuffer, u32 offset);

		void SetViewport(const Viewport& v);

		void SetViewMatrix(const Matrix4x4f& m);

		void SetProjectionMatrix(const Matrix4x4f& m);

		void SetWorldMatrix(const Matrix4x4f& m);

		void UpdateUITransform();

		void SetMaterial(MaterialId materialId);

		void BindActiveMaterial(ID3D11DeviceContext* pDC);

		void UpdateMaterialConstants(MaterialId materialId, const void* pData, size_t size);

		void UpdateBuffer(BufferId bufferId, const void* pData, size_t size);

		void UpdateTexture(TextureId textureId, u32 mipId, u32 sliceId, const void* pData, size_t rowPitchBytes);

		void BindGlobalTexture(u32 slot, TextureId textureId, SamplerId samplerId, ShaderStageFlag::Type stageBinding);

		void BindGlobalBufferCBV(u32 slot, BufferId bufferId, ShaderStageFlag::Type stageBinding);

		void BindGlobalBufferSRV(u32 slot, BufferId bufferId, ShaderStageFlag::Type stageBinding);

		void BindGlobalBufferUAV(u32 slot, BufferId bufferId, ShaderStageFlag::Type stageBinding);

		void BindFrameConstants();

		void BindUIFrameConstants();

		TextureId GetTransientSurface(SurfaceSize surfaceSize, TextureFormat format);

		SurfaceSize GetDisplaySurfaceSize() const;

		SurfaceSize GetRenderTargetSize() const;

		void SetRenderTargets(const TextureId* colourTargetIdArray, u32 mrtCount, TextureId depthTextureId);

		void SetRenderTargetsToSwapChain(bool bEnableDefaultDepth);

		void SetRenderTargetsToSwapChain(TextureId depthTextureId);

		void ClearDepthTarget(TextureId textureId, f32 depthValue);

		void ClearRenderTarget(TextureId textureId, ColourValue clearColour);

		ShaderId GetMaterialShader(MaterialShaderKey key);

		void SetLightPosition(u32 index, const Vector3f& vPosition);

		void SetLightDirection(u32 index, const Vector3f& vDirection);

		void SetLightColour(u32 index, ColourValue colour);

		void SetAmbientLight(ColourValue colour);

		void SetDefaultRenderTargetClearColour(ColourValue colour);

		void QueueGenerateMips(ID3D11ShaderResourceView* pSRV);

		void RegisterWindowCallback(WindowCallback callback);

		void RegisterGraphicsCallbacks(IGraphicsCallbacks* pCallbacks);

		result_t LoadImageAsRGBA(const char* pFilePath, std::vector<u8>& imageOut, SurfaceSize& surfaceSizeOut);

		void BindComputeShader(ShaderId shaderId);

		void Dispatch(u32 groupCountX, u32 groupCountY, u32 groupCountZ);

		void UpdateConstantBuffers();

		float GetGPUFrameTime() const { return m_gpuFrameTimeMS; }

		void PushMarker(const char* name);

		void PopMarker();
	private:
		Graphics_Impl(const SystemDesc& rDesc);
		~Graphics_Impl();

		result_t InitWindow(const SystemDesc& rDesc);

		result_t ReleaseWindow();

		result_t InitDirectX();

		result_t InitTimestamps();

		result_t RecreateRenderTargetView();

		result_t RecreateDepthBuffer();

		result_t CompileInternalShaders();

		void CompileMaterialShader(MaterialShaderKey& key);

		result_t CreateInputLayouts();

		result_t CreatePipelineState();

		bool UpdateMessageLoop();

		void InternalBindMaterial(Material* pMaterial, ID3D11DeviceContext* pDC);

		void InternalBindMesh(const Mesh* pMesh, ID3D11DeviceContext* pDC);

		result_t Resize(u32 width, u32 height);

		static LRESULT CALLBACK MainWndProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam);
	private:
		static Graphics_Impl* ms_pInstance;

		HINSTANCE m_hInstance = NULL;
		HWND m_hWnd = NULL;

		std::vector<WindowCallback> m_wndCallbacks;
		std::vector<IGraphicsCallbacks*> m_graphicsCallbacks;

		ComPtr<ID3D11Device> m_pDevice;
		ComPtr<ID3D11DeviceContext> m_pDeviceContext;
		ComPtr<IDXGIDevice> m_pDXGIDevice;
		ComPtr<IDXGIFactory2> m_pDXGIFactory;
		ComPtr<IDXGIAdapter> m_pDXGIAdapter;
		ComPtr<IDXGIOutput> m_pDXGIOutput;
		ComPtr<IDXGISwapChain1> m_pSwapChain;
		ComPtr<ID3D11Texture2D> m_pDefaultDepthBuffer;
		ComPtr<ID3D11RenderTargetView> m_pBackBufferRTV;
		ComPtr<ID3D11DepthStencilView> m_pDefaultDSV;

		ComPtr<ID3D11InputLayout> m_pMeshInputLayout;
		ComPtr<ID3D11InputLayout> m_pSkinnedMeshInputLayout;

		struct FrameConstantData
		{
			Matrix4x4f viewMtx;
			Matrix4x4f projectionMtx;
			Matrix4x4f viewProjectionMtx;
			Matrix4x4f invViewMtx;
			Matrix4x4f invProjectionMtx;
			alignas(16) Vector4f viewPosition;
			alignas(16) float time[4];
		};

		ShaderConstantBuffer_Impl<FrameConstantData> m_frameConstants;

		struct DrawConstantData
		{
			Matrix4x4f mvpMtx;
			Matrix4x4f worldMtx;
			Matrix4x4f normalMtx;
		};
		ShaderConstantBuffer_Impl<DrawConstantData> m_drawConstants;

		struct LightConstantData
		{
			Vector4f lightPos[MaterialShaderKey::kMaxLights];
			Vector4f lightDir[MaterialShaderKey::kMaxLights];
			Vector4f lightColour[MaterialShaderKey::kMaxLights];
			Vector4f lightAmbient;
		};
		ShaderConstantBuffer_Impl<LightConstantData> m_lightConstants;

		ShaderConstantBuffer_Impl<MaterialConstantData> m_materialConstants;

		struct UIFrameConstantData
		{
			Matrix4x4f viewProjectionMtx;
			Vector4f viewportRect;
		};
		ShaderConstantBuffer_Impl<UIFrameConstantData> m_uiFrameConstants;

		u32 m_nSurfaceWidth = 0;
		u32 m_nSurfaceHeight = 0;

		u32 m_nRenderTargetWidth = 0;
		u32 m_nRenderTargetHeight = 0;

		ShaderId m_meshShaders[MaterialShaderKey::kPermutations];
		ShaderId m_skinnedMeshVS;

		MaterialId m_activeMaterial;
		Material* m_pPrevMaterial = nullptr;
		const Mesh* m_pPrevMesh = nullptr;

		ComPtr<ID3D11RasterizerState> m_pFallbackRasterState;
		ComPtr<ID3D11DepthStencilState> m_pFallbackDepthStencilState;

		ComPtr<ID3D11BlendState> m_pBlendStateOpaque;
		ComPtr<ID3D11BlendState> m_pBlendStateAdditive;
		ComPtr<ID3D11BlendState> m_pBlendStateAlphaBlend;

		std::vector<ID3D11ShaderResourceView*> m_mipQueue;

		f32 m_defaultRenderTargetClearColour[4];

		struct SurfaceKey
		{
			union {
				struct
				{
					u32 m_width	 : 16;
					u32 m_height : 16;
					u32 m_format : 4;
				} m_bits;
				u32 m_value;
			};
			SurfaceKey() = default;
			SurfaceKey(SurfaceSize size, TextureFormat format)
				: m_value(0)
			{
				m_bits.m_width = (u32)size.m_width;
				m_bits.m_height = (u32)size.m_height;
				m_bits.m_format = (u32)format;
			}
			bool operator==(const SurfaceKey& rhs) const { return m_value == rhs.m_value; }
			bool operator!=(const SurfaceKey& rhs) const { return m_value != rhs.m_value; }
			bool operator<(const SurfaceKey& rhs) const { return m_value < rhs.m_value; }
		};

		struct TransientSurface
		{
			SurfaceKey m_key;
			TextureId m_textureId;
			u64 m_timestamp;
		};

		std::vector<TransientSurface> m_transientSurfaces;

		ComPtr<ID3D11Query> m_pGPUBeginFrameTime;
		ComPtr<ID3D11Query> m_pGPUEndFrameTime;
		ComPtr<ID3D11Query> m_pGPUDisjointQuery;
		ComPtr<ID3DUserDefinedAnnotation> m_pAnnotation;

		u64 m_frameTimestamp = 0;
		f32 m_gpuFrameTimeMS = 0.f;
	};
};

namespace Play3d::Graphics
{
	UINT TranslateBufferBindFlags(BufferBindFlags::Type bindFlags)
	{
		static UINT kBitsTable[BufferBindFlags::kMaxBits] = {D3D11_BIND_FLAG::D3D11_BIND_CONSTANT_BUFFER,
															 D3D11_BIND_FLAG::D3D11_BIND_VERTEX_BUFFER,
															 D3D11_BIND_FLAG::D3D11_BIND_INDEX_BUFFER,
															 D3D11_BIND_FLAG::D3D11_BIND_SHADER_RESOURCE,
															 D3D11_BIND_FLAG::D3D11_BIND_UNORDERED_ACCESS,
		};

		UINT ret = 0;
		for (u32 i = 0; i < BufferBindFlags::kMaxBits; ++i)
		{
			const BufferBindFlags::Type flag = 1 << i;
			if (bindFlags & flag)
			{
				ret |= kBitsTable[i];
			}
		};
		return ret;
	}

	D3D11_USAGE TranslateBufferUsage(BufferFlags::Type flags)
	{
		if (flags & BufferFlags::DYNAMIC)
		{
			return D3D11_USAGE_DYNAMIC;
		}
		if (flags & BufferFlags::IMMUTABLE)
		{
			return D3D11_USAGE_IMMUTABLE;
		}
		return D3D11_USAGE_DEFAULT;
	}

	Buffer::Buffer(const BufferDesc& rDesc)
	{
		ID3D11Device* pDevice = Graphics::Graphics_Impl::Instance().GetDevice();

		D3D11_BUFFER_DESC desc = {};
		desc.ByteWidth = (u32)rDesc.m_sizeBytes;
		desc.Usage = TranslateBufferUsage(rDesc.m_flags);
		desc.BindFlags = TranslateBufferBindFlags(rDesc.m_bindFlags);

		desc.CPUAccessFlags = 0;
		if (rDesc.m_flags & BufferFlags::DYNAMIC)
		{
			desc.CPUAccessFlags |= D3D11_CPU_ACCESS_WRITE;
		}

		desc.MiscFlags = 0;
		desc.StructureByteStride = 0;
		if (rDesc.m_flags & BufferFlags::STRUCTURED)
		{
			desc.MiscFlags |= D3D11_RESOURCE_MISC_BUFFER_STRUCTURED;
			desc.StructureByteStride = rDesc.m_structureStrideBytes;
		}

		if (rDesc.m_flags & BufferFlags::INDIRECT_ARGS)
		{
			desc.MiscFlags |= D3D11_RESOURCE_MISC_DRAWINDIRECT_ARGS;
		}

		D3D11_SUBRESOURCE_DATA data = {};
		data.pSysMem = rDesc.m_pInitialData;

		HRESULT hr = pDevice->CreateBuffer(&desc, rDesc.m_pInitialData ? &data : nullptr, &m_pBuffer);
		PLAY_ASSERT(SUCCEEDED(hr));

		if (rDesc.m_pDebugName)
		{
			m_pBuffer->SetPrivateData(WKPDID_D3DDebugObjectName, (UINT)strlen(rDesc.m_pDebugName), rDesc.m_pDebugName);
		}
	}

	ID3D11Buffer* Buffer::GetBuffer()
	{
		return m_pBuffer.Get();
	}

	ID3D11ShaderResourceView* Buffer::GetSRV()
	{
		if (!m_pSRV)
		{
			ID3D11Device* pDevice = Graphics::Graphics_Impl::Instance().GetDevice();
			HRESULT hr = pDevice->CreateShaderResourceView(m_pBuffer.Get(), nullptr, &m_pSRV);
			PLAY_ASSERT(SUCCEEDED(hr));
		}

		return m_pSRV.Get();
	}

	ID3D11UnorderedAccessView* Buffer::GetUAV()
	{
		if (!m_pUAV)
		{
			ID3D11Device* pDevice = Graphics::Graphics_Impl::Instance().GetDevice();
			HRESULT hr = pDevice->CreateUnorderedAccessView(m_pBuffer.Get(), nullptr, &m_pUAV);
			PLAY_ASSERT(SUCCEEDED(hr));
		}

		return m_pUAV.Get();
	}

}

//------------------------------------------ Graphics/GraphicsApi.cpp ------------------------------------------

namespace Play3d::Graphics
{
	struct InternalState
	{
		PrimitiveBatch* m_pCurrentPrimitiveBatch;
	};

	static InternalState s_internalState;

	TextureId GetTransientSurface(SurfaceSize surfaceSize, TextureFormat format)
	{
		return Graphics_Impl::Instance().GetTransientSurface(surfaceSize, format);
	}

	SurfaceSize GetDisplaySurfaceSize()
	{
		return {Graphics_Impl::Instance().GetDisplaySurfaceSize()};
	}

	void SetRenderTargets(TextureId colourTargetId, TextureId depthTextureId)
	{
		TextureId colourTargets[] = {colourTargetId};
		Graphics_Impl::Instance().SetRenderTargets(colourTargets, 1, depthTextureId);
	}

	void SetRenderTargets(const TextureId* colourTargetIdArray, u32 mrtCount, TextureId depthTextureId)
	{
		Graphics_Impl::Instance().SetRenderTargets(colourTargetIdArray, mrtCount, depthTextureId);
	}

	void SetRenderTargetsToSwapChain(bool bEnableDefaultDepth)
	{
		Graphics_Impl::Instance().SetRenderTargetsToSwapChain(bEnableDefaultDepth);
	}

	void SetRenderTargetsToSwapChain(TextureId depthTextureId)
	{
		Graphics_Impl::Instance().SetRenderTargetsToSwapChain(depthTextureId);
	}

	void ClearDepthTarget(TextureId textureId, f32 depthValue /*= 1.0f*/)
	{
		Graphics_Impl::Instance().ClearDepthTarget(textureId, depthValue);
	}

	void ClearRenderTarget(TextureId textureId, ColourValue clearColour)
	{
		Graphics_Impl::Instance().ClearRenderTarget(textureId, clearColour);
	}

	void SetViewport(const Viewport& viewport)
	{
		Graphics_Impl::Instance().SetViewport(viewport);
	}

	void SetViewMatrix(const Matrix4x4f& m)
	{
		Graphics_Impl::Instance().SetViewMatrix(m);
	}

	void SetProjectionMatrix(const Matrix4x4f& m)
	{
		Graphics_Impl::Instance().SetProjectionMatrix(m);
	}

	void BeginPrimitiveBatch()
	{
		if (!s_internalState.m_pCurrentPrimitiveBatch)
		{
			s_internalState.m_pCurrentPrimitiveBatch = PrimitiveRenderSystem::Instance().AllocatePrimitiveBatch();
		}
	}

	void DrawPoint(const Vector3f& v1, ColourValue colour)
	{
		PLAY_ASSERT(s_internalState.m_pCurrentPrimitiveBatch);
		s_internalState.m_pCurrentPrimitiveBatch->AppendPoint(v1, colour);
	}

	void DrawLine(const Vector3f& v1, const Vector3f& v2, ColourValue colour)
	{
		PLAY_ASSERT(s_internalState.m_pCurrentPrimitiveBatch);
		s_internalState.m_pCurrentPrimitiveBatch->AppendLine(v1, v2, colour, colour);
	}

	void DrawLine(const Vector3f& v1, const Vector3f& v2, ColourValue c1, ColourValue c2)
	{
		PLAY_ASSERT(s_internalState.m_pCurrentPrimitiveBatch);
		s_internalState.m_pCurrentPrimitiveBatch->AppendLine(v1, v2, c1, c2);
	}

	void DrawTriangle(const Vector3f& v1, const Vector3f& v2, const Vector3f& v3, ColourValue colour)
	{
		PLAY_ASSERT(s_internalState.m_pCurrentPrimitiveBatch);
		s_internalState.m_pCurrentPrimitiveBatch->AppendTriangle(v1, v2, v3, colour, colour, colour);
	}

	void DrawTriangle(const Vector3f& v1, const Vector3f& v2, const Vector3f& v3, ColourValue c1, ColourValue c2,
					  ColourValue c3)
	{
		PLAY_ASSERT(s_internalState.m_pCurrentPrimitiveBatch);
		s_internalState.m_pCurrentPrimitiveBatch->AppendTriangle(v1, v2, v3, c1, c2, c3);
	}

	void DrawQuad(const Vector3f& v1, const Vector3f& v2, const Vector3f& v3, const Vector3f& v4, ColourValue colour)
	{
		PLAY_ASSERT(s_internalState.m_pCurrentPrimitiveBatch);
		s_internalState.m_pCurrentPrimitiveBatch->AppendTriangle(v1, v2, v3, colour, colour, colour);
		s_internalState.m_pCurrentPrimitiveBatch->AppendTriangle(v2, v4, v3, colour, colour, colour);
	}

	void DrawQuad(const Vector3f& v1, const Vector3f& v2, const Vector3f& v3, const Vector3f& v4, ColourValue c1,
				  ColourValue c2, ColourValue c3, ColourValue c4)
	{
		PLAY_ASSERT(s_internalState.m_pCurrentPrimitiveBatch);
		s_internalState.m_pCurrentPrimitiveBatch->AppendTriangle(v1, v2, v3, c1, c2, c3);
		s_internalState.m_pCurrentPrimitiveBatch->AppendTriangle(v2, v4, v3, c2, c4, c3);
	}

	void EndPrimitiveBatch()
	{
		if (s_internalState.m_pCurrentPrimitiveBatch)
		{
			PrimitiveRenderSystem::Instance().DrawPrimitiveBatch(s_internalState.m_pCurrentPrimitiveBatch);
			s_internalState.m_pCurrentPrimitiveBatch = nullptr;
		}
	}

	void DrawMesh(MeshId hMesh, const Matrix4x4f& transform, u32 elementOffset, u32 elementCount)
	{
		Graphics_Impl::Instance().SetWorldMatrix(transform);

		Mesh* pMesh = Resources::ResourceManager<Mesh>::Instance().GetPtr(hMesh);
		if (pMesh)
		{
			Graphics_Impl::Instance().DrawMesh(pMesh, elementOffset, elementCount);
		}
	}

	void DrawInstancedMesh(MeshId hMesh, u32 kInstanceCount, u32 kInstanceOffset, u32 elementOffset,
						   u32 elementCount)
	{
		Mesh* pMesh = Resources::ResourceManager<Mesh>::Instance().GetPtr(hMesh);
		if (pMesh)
		{
			Graphics_Impl::Instance().DrawInstancedMesh(pMesh,
														kInstanceCount,
														kInstanceOffset,
														elementOffset,
														elementCount);
		}
	}

	void DrawWithoutVertices(u32 elements)
	{
		Graphics_Impl::Instance().DrawWithoutVertices(elements);
	}

	void DrawIndirectWithoutVertices(BufferId hArgBuffer, u32 offset)
	{
		Graphics_Impl::Instance().DrawIndirectWithoutVertices(hArgBuffer, offset);
	}

	void SetMaterial(MaterialId materialId)
	{
		Graphics_Impl::Instance().SetMaterial(materialId);
	}

	void UpdateMaterialConstants(MaterialId materialId, const void* pData, size_t size)
	{
		Graphics_Impl::Instance().UpdateMaterialConstants(materialId, pData, size);
	}

	void UpdateTexture(TextureId textureId, u32 mipId, u32 sliceId, const void* pData, size_t rowPitchBytes)
	{
		PLAY_ASSERT(pData && rowPitchBytes > 0);
		Graphics_Impl::Instance().UpdateTexture(textureId, mipId, sliceId, pData, rowPitchBytes);
	}

	void BindGlobalTexture(u32 slot, TextureId textureId, SamplerId samplerId, ShaderStageFlag::Type stageBinding)
	{
		PLAY_ASSERT(slot >= kGlobalTextureSlotStart);
		Graphics_Impl::Instance().BindGlobalTexture(slot, textureId, samplerId, stageBinding);
	}

	void UpdateBuffer(BufferId bufferId, const void* pData, size_t size)
	{
		PLAY_ASSERT(pData && size > 0);
		Graphics_Impl::Instance().UpdateBuffer(bufferId, pData, size);
	}

	void BindGlobalBuffer(u32 slot, BufferId bufferId, ShaderStageFlag::Type stageBinding,
						  BufferBindFlags::Type bindFlags)
	{
		if (bindFlags & BufferBindFlags::CONSTANT)
		{
			PLAY_ASSERT(slot >= kGlobalBufferSlotStart);
			Graphics_Impl::Instance().BindGlobalBufferCBV(slot, bufferId, stageBinding);
		}

		if (bindFlags & BufferBindFlags::SRV)
		{
			Graphics_Impl::Instance().BindGlobalBufferSRV(slot, bufferId, stageBinding);
		}

		if (bindFlags & BufferBindFlags::UAV)
		{
			Graphics_Impl::Instance().BindGlobalBufferUAV(slot, bufferId, stageBinding);
		}
	}

	void SetLightPosition(u32 index, const Vector3f& vPosition)
	{
		Graphics_Impl::Instance().SetLightPosition(index, vPosition);
	}

	void SetLightDirection(u32 index, const Vector3f& vDirection)
	{
		Graphics_Impl::Instance().SetLightDirection(index, vDirection);
	}

	void SetLightColour(u32 index, ColourValue colour)
	{
		Graphics_Impl::Instance().SetLightColour(index, colour);
	}

	void SetAmbientLight(ColourValue colour)
	{
		Graphics_Impl::Instance().SetAmbientLight(colour);
	}

	void SetDefaultRenderTargetClearColour(ColourValue colour)
	{
		Graphics_Impl::Instance().SetDefaultRenderTargetClearColour(colour);
	}

	MeshId CreatePlane(f32 fHalfSizeX, f32 fHalfSizeZ, ColourValue colour /*= Colour::White*/, f32 fUVScale /*= 1.0f*/)
	{
		MeshBuilder builder;
		builder.AddVertex(Vector3f(-fHalfSizeX, 0, -fHalfSizeZ), Vector3f(0, 1, 0), Vector2f(0, 0), colour);
		builder.AddVertex(Vector3f(fHalfSizeX, 0, -fHalfSizeZ), Vector3f(0, 1, 0), Vector2f(fUVScale, 0), colour);
		builder.AddVertex(Vector3f(fHalfSizeX, 0, fHalfSizeZ), Vector3f(0, 1, 0), Vector2f(fUVScale, fUVScale), colour);
		builder.AddVertex(Vector3f(-fHalfSizeX, 0, fHalfSizeZ), Vector3f(0, 1, 0), Vector2f(0, fUVScale), colour);
		builder.AddQuad(0, 1, 2, 3);
		builder.GenerateTangents();
		return builder.CreateMesh();
	}

	MeshId CreatePlaneXY(f32 fWidthX, f32 fHeightY, ColourValue colour /*= Colour::White*/, f32 fUVScale /*= 1.0f*/)
	{
		MeshBuilder builder;
		builder.AddVertex(Vector3f(0, 0, 0), Vector3f(0, 1, 0), Vector2f(0, 0), colour);
		builder.AddVertex(Vector3f(fWidthX, 0, 0), Vector3f(0, 1, 0), Vector2f(fUVScale, 0), colour);
		builder.AddVertex(Vector3f(fWidthX, fHeightY, 0), Vector3f(0, 1, 0), Vector2f(fUVScale, fUVScale), colour);
		builder.AddVertex(Vector3f(0, fHeightY, 0), Vector3f(0, 1, 0), Vector2f(0, fUVScale), colour);
		builder.AddQuad(0, 1, 2, 3);
		builder.GenerateTangents();
		return builder.CreateMesh();
	}

	MeshId CreateMeshCube(f32 size, ColourValue colour)
	{
		return CreateMeshBox(size, size, size, colour);
	}

	MeshId CreateMeshBox(f32 sizeX, f32 sizeY, f32 sizeZ, ColourValue colour)
	{
		const f32 x = sizeX / 2.f;
		const f32 y = sizeY / 2.f;
		const f32 z = sizeZ / 2.f;

		Vector3f positions[] = {Vector3f(-x, -y, -z),
								Vector3f(x, -y, -z),
								Vector3f(x, y, -z),
								Vector3f(-x, y, -z),

								Vector3f(-x, -y, z),
								Vector3f(x, -y, z),
								Vector3f(x, y, z),
								Vector3f(-x, y, z)};

		MeshBuilder builder;

		u32 indices[] = {
				0, 3, 2, 1,
				0, 1, 5, 4,
				0, 4, 7, 3,
				6, 5, 1, 2,
				6, 2, 3, 7,
				6, 7, 4, 5,
		};
		builder.AddIndexedFlatFace(4, indices, positions, colour);
		builder.AddIndexedFlatFace(4, indices + 4, positions, colour);
		builder.AddIndexedFlatFace(4, indices + 8, positions, colour);
		builder.AddIndexedFlatFace(4, indices + 12, positions, colour);
		builder.AddIndexedFlatFace(4, indices + 16, positions, colour);
		builder.AddIndexedFlatFace(4, indices + 20, positions, colour);

		builder.GenerateTangents();

		return builder.CreateMesh();
	}

	MeshId CreateCylinder(f32 fLength, f32 fRadius, u32 segments, ColourValue colour /*= Colour::White*/)
	{
		std::vector<Vector3f> shape(segments);
		f32 fStep = kfTwoPi / segments;
		for (u32 i = 0; i < segments; ++i)
		{
			f32 theta = -fStep * i;
			shape[i] = Vector3f(sin(theta) * fRadius, 0.f, cos(theta) * fRadius);
		}
		MeshBuilder builder;
		builder.Extrude(segments, shape.data(), colour, fLength);

		builder.GenerateTangents();

		return builder.CreateMesh();
	}

	MeshId CreateSphere(f32 fRadius, u32 segments, u32 slices, ColourValue colour /*= Colour::White*/)
	{
		MeshBuilder builder;
		const f32 r0 = kfPi / (slices + 1);
		const f32 r1 = kfTwoPi / segments;

		for (u32 i = 0; i < slices; ++i)
		{
			f32 phi = r0 * (i + 1);
			f32 sphi = sin(phi);
			f32 cphi = cos(phi);

			for (u32 j = 0; j < segments; ++j)
			{
				f32 theta = r1 * (j + 1);
				f32 stheta = sin(theta);
				f32 ctheta = cos(theta);
				Vector3f N(ctheta * sphi, cphi, stheta * sphi);
				builder.AddVertex(N * fRadius, N, Vector2f(0, 0), colour);
			}
		}

		u32 iCap0 = builder.AddVertex(Vector3f(0, fRadius, 0), Vector3f(0, 1.f, 0), Vector2f(0, 0), colour);
		u32 iCapN = builder.AddVertex(Vector3f(0, -fRadius, 0), Vector3f(0, -1.f, 0), Vector2f(0, 0), colour);

		for (u32 i = 0; i < (slices - 1); ++i)
		{
			for (u32 j = 0; j < segments; ++j)
			{
				u32 iA = i * segments + j;
				u32 iB = (i + 1) * segments + j;
				u32 iC = (i + 1) * segments + ((j + 1) % segments);
				u32 iD = i * segments + ((j + 1) % segments);
				builder.AddQuad(iA, iB, iC, iD);
			}
		}

		for (u32 i = 0; i < segments; ++i)
		{
			u32 iA = iCap0;
			u32 iB = i;
			u32 iC = (i + 1) % segments;
			builder.AddTriangle(iA, iB, iC);
		}

		u32 iEndCap0 = iCapN - segments - 1;
		for (u32 i = 0; i < segments; ++i)
		{
			u32 iA = iCapN;
			u32 iB = iEndCap0 + i;
			u32 iC = iEndCap0 + ((i + 1) % segments);
			builder.AddTriangle(iA, iC, iB);
		}

		builder.GenerateTangents();

		return builder.CreateMesh();
	}

	MeshId CreatePlatonicTetrahedron(ColourValue colour /*= Colour::White*/)
	{
		const f32 a = sqrt(2.0f) / 3.0f;
		const f32 b = sqrt(6.0f) / 3.0f;
		const f32 c = -1.f / 3.f;

		Vector3f positions[] = {
			Vector3f(0, 0, 1),
			Vector3f(2.f * a, 0, c),
			Vector3f(-a, b, c),
			Vector3f(-a, -b, c),
		};

		MeshBuilder builder;

		u32 indices[] = {
				0, 1, 2,
				0, 2, 3,
				0, 3, 1,
				1, 3, 2,
		};

		builder.AddIndexedFlatFace(3, indices, positions, colour);
		builder.AddIndexedFlatFace(3, indices + 3, positions, colour);
		builder.AddIndexedFlatFace(3, indices + 6, positions, colour);
		builder.AddIndexedFlatFace(3, indices + 9, positions, colour);

		builder.GenerateTangents();

		return builder.CreateMesh();
	}

	MeshId CreatePlatonicHexahedron(ColourValue colour /*= Colour::White*/)
	{
		return CreateMeshCube(2.0f / sqrt(3.f));
	}

	Play3d::Graphics::MeshId CreatePlatonicOctahedron(ColourValue colour /*= Colour::White*/)
	{
		Vector3f positions[] = {
			Vector3f(1, 0, 0),
			Vector3f(-1, 0, 0),
			Vector3f(0, 1, 0),
			Vector3f(0, -1, 0),
			Vector3f(0, 0, 1),
			Vector3f(0, 0, -1),
		};

		MeshBuilder builder;

		u32 indices[] = {
				4,0,2,
				4,2,1,
				4,1,3,
				4,3,0,

				5,2,0,
				5,1,2,
				5,3,1,
				5,0,3,
		};

		builder.AddIndexedFlatFace(3, indices, positions, colour);
		builder.AddIndexedFlatFace(3, indices + 3, positions, colour);
		builder.AddIndexedFlatFace(3, indices + 6, positions, colour);
		builder.AddIndexedFlatFace(3, indices + 9, positions, colour);
		builder.AddIndexedFlatFace(3, indices + 12, positions, colour);
		builder.AddIndexedFlatFace(3, indices + 15, positions, colour);
		builder.AddIndexedFlatFace(3, indices + 18, positions, colour);
		builder.AddIndexedFlatFace(3, indices + 21, positions, colour);

		builder.GenerateTangents();

		return builder.CreateMesh();
	}

	MeshId CreateMeshFromObjString(std::string_view objString, ColourValue colour, f32 fScale)
	{
		MeshBuilder builder;
		result_t result = builder.ParseObjFormat(objString, colour, fScale);
		if (RESULT_OK == result)
		{
			builder.GenerateTangents();
			return builder.CreateMesh();
		}

		return MeshId();
	}

	MeshId CreateMeshFromObjFile(const char* filePath, ColourValue colour, f32 fScale)
	{
		if (!System::CheckFileExists(filePath))
		{
			Debug::Printf("ERROR: Obj file does not exist! path='%s'\n", filePath);
			return MeshId();
		}

		MeshBuilder builder;
		result_t result = RESULT_FAIL;

		size_t sizeBytes = 0;
		const void* pFileData = System::LoadFileData(filePath, sizeBytes);
		if (pFileData && sizeBytes)
		{
			result = builder.ParseObjFormat(std::string_view(static_cast<const char*>(pFileData), sizeBytes), colour, fScale);
		}

		if (RESULT_OK == result)
		{
			builder.GenerateTangents();
			return builder.CreateMesh();
		}
		return MeshId();
	}

	TextureId CreateTextureCheckerboard(u32 width, u32 height, ColourValue a, ColourValue b, u32 checkSize)
	{
		const u32 rowPitch = width;
		u32* pImageData = new u32[height * rowPitch];

		for (u32 i = 0; i < height; ++i)
		{
			u32 ti = (i / checkSize) % 2;
			for (u32 j = 0; j < width; ++j)
			{
				u32 tj = (j / checkSize + ti) % 2;
				pImageData[i * rowPitch + j] = tj ? a.as_u32() : b.as_u32();
			}
		}

		TextureDesc desc;
		desc.m_width = width;
		desc.m_height = height;
		desc.m_format = TextureFormat::RGBA;
		desc.m_pImageData = pImageData;
		TextureId id = Resources::CreateAsset<Texture>(desc);

		delete[] pImageData;

		return id;
	}

	TextureId CreateTextureFromFile(const char* pFilePath, bool bGenerateMipLevels)
	{
		TextureId textureId;

		std::vector<u8> imageData;
		SurfaceSize surfaceSize;
		result_t result = Graphics::Graphics_Impl::Instance().LoadImageAsRGBA(pFilePath, imageData, surfaceSize);

		if (Result::RESULT_OK == result)
		{
			TextureDesc desc;
			desc.m_width = surfaceSize.m_width;
			desc.m_height = surfaceSize.m_height;
			desc.m_format = TextureFormat::RGBA;
			desc.m_pImageData = imageData.data();
			desc.m_pDebugName = pFilePath;
			desc.m_flags = TextureFlags::ENABLE_TEXTURE;
			if (bGenerateMipLevels)
			{
				desc.m_flags |= TextureFlags::GENERATE_MIPS;
			}
			textureId = Resources::CreateAsset<Texture>(desc);
		}

		return textureId;
	}

	Play3d::Graphics::TextureId CreateTextureArrayFromFiles(std::initializer_list<const char*> pFilePaths)
	{
		TextureId textureId;
		TextureArrayDesc desc;
		desc.m_format = TextureFormat::RGBA;
		bool FirstTextureLoaded = false;

		for (const char* path : pFilePaths)
		{
			std::vector<u8> imageData;
			SurfaceSize surfaceSize;
			result_t result = Graphics::Graphics_Impl::Instance().LoadImageAsRGBA(path, imageData, surfaceSize);
			PLAY_ASSERT(Result::RESULT_OK == result);
			if (!FirstTextureLoaded)
			{
				desc.m_width = surfaceSize.m_width;
				desc.m_height = surfaceSize.m_height;
				FirstTextureLoaded = true;
			}
			else
			{
				PLAY_ASSERT_MSG(surfaceSize.m_width == desc.m_width && surfaceSize.m_height == desc.m_height,
								"Texture sizes in Texture array not equal, images must be identical in dimensions!");
			}
			desc.m_ppImageData.push_back(imageData);
		}
		textureId = Resources::CreateAsset<Texture>(desc);

		return textureId;
	}

	result_t LoadImageAsRGBA(const char* path, std::vector<u8>& imageOut, SurfaceSize& surfaceSizeOut)
	{
		return Graphics::Graphics_Impl::Instance().LoadImageAsRGBA(path, imageOut, surfaceSizeOut);
	}

	SamplerId CreateLinearSampler()
	{
		SamplerDesc desc;
		desc.m_filter = FilterMode::BILINEAR;
		desc.SetUniformAddressMode(AddressMode::WRAP);
		return Resources::CreateAsset<Graphics::Sampler>(desc);
	}

	SamplerId CreatePointSampler()
	{
		SamplerDesc desc;
		desc.m_filter = FilterMode::POINT;
		desc.SetUniformAddressMode(AddressMode::WRAP);
		return Resources::CreateAsset<Graphics::Sampler>(desc);
	}

	void BindComputeShader(ShaderId shaderId)
	{
		PLAY_ASSERT(shaderId.IsValid());
		Graphics_Impl::Instance().BindComputeShader(shaderId);
	}

	void Dispatch(u32 groupCountX, u32 groupCountY, u32 groupCountZ)
	{
		PLAY_ASSERT(groupCountX > 0 && groupCountY > 0 && groupCountZ > 0);
		Graphics_Impl::Instance().Dispatch(groupCountX, groupCountY, groupCountZ);
	}

	float GetGPUFrameTime()
	{
		return Graphics_Impl::Instance().GetGPUFrameTime();
	}

	void PushMarker(const char* name)
	{
		Graphics_Impl::Instance().PushMarker(name);
	}

	void PopMarker()
	{
		Graphics_Impl::Instance().PopMarker();
	}

	void RegisterWindowCallback(WindowCallback callback)
	{
		Graphics_Impl::Instance().RegisterWindowCallback(callback);
	}

	HWND GetWindowHandle()
	{
		return Graphics_Impl::Instance().GetHWnd();
	}

	void SetFullscreenMode(bool bEnable)
	{
		return Graphics_Impl::Instance().SetFullscreenMode(bEnable);
	}

	ID3D11Device* GetDevice()
	{
		return Graphics_Impl::Instance().GetDevice();
	}

	ID3D11DeviceContext* GetDeviceContext()
	{
		return Graphics_Impl::Instance().GetDeviceContext();
	}

}

//----------------------------------------- Graphics/Graphics_Impl.cpp -----------------------------------------

//----------------------------------------- Graphics/ShaderCode_Impl.h -----------------------------------------

namespace Play3d::Graphics
{
extern const char* HLSL_FontShader;
extern const char* HLSL_MeshShader;
extern const char* HLSL_PrimitiveBatchShader;
extern const char* HLSL_SkinnedMeshShader;
extern const char* HLSL_SpriteBatchShader;
extern const char* HLSL_StandardMacros;

}

#pragma comment(lib, "d3d11.lib")
#pragma comment(lib, "dxguid.lib")
#pragma comment(lib, "windowscodecs.lib")
namespace Play3d::Graphics
{
	Graphics_Impl* Graphics_Impl::ms_pInstance = nullptr;

	bool Graphics_Impl::UpdateMessageLoop()
	{
		MSG msg;
		while (PeekMessage(&msg, NULL, 0, 0, PM_REMOVE))
		{
			if (msg.message == WM_QUIT)
			{
				return true;
			}

			TranslateMessage(&msg);
			DispatchMessage(&msg);
		}
		return false;
	}

	void Graphics_Impl::UpdateConstantBuffers()
	{
		ID3D11DeviceContext* pDC(m_pDeviceContext.Get());
		m_frameConstants.UpdateGPU(pDC);
		m_drawConstants.UpdateGPU(pDC);
		m_materialConstants.UpdateGPU(pDC);
		m_lightConstants.UpdateGPU(pDC);
		m_uiFrameConstants.UpdateGPU(pDC);
	}

	void Graphics_Impl::Initialise(const SystemDesc& rDesc)
	{
		if (!ms_pInstance)
		{
			ms_pInstance = new Graphics_Impl(rDesc);
		}
	}

	void Graphics_Impl::Destroy()
	{
		ComPtr<ID3D11Debug> pDebug;
		HRESULT hr = ms_pInstance->m_pDevice.As(&pDebug);

		delete ms_pInstance;
		ms_pInstance = nullptr;

		if (SUCCEEDED(hr))
		{
			pDebug->ReportLiveDeviceObjects(D3D11_RLDO_DETAIL | D3D11_RLDO_IGNORE_INTERNAL);
		}
	}

	Graphics_Impl::Graphics_Impl(const SystemDesc& rDesc)
		: m_hInstance(GetModuleHandle(NULL))
		, m_hWnd(NULL)
		, m_nSurfaceWidth(0)
		, m_nSurfaceHeight(0)
		, m_frameTimestamp(0)
		, m_defaultRenderTargetClearColour{0.0f, 0.0f, 0.0f, 1.0f}
	{
		InitWindow(rDesc);
		InitDirectX();
	}

	Graphics_Impl::~Graphics_Impl()
	{
		Flush();
		ReleaseWindow();
	}

	result_t Graphics_Impl::PostInitialise()
	{
		CompileInternalShaders();
		CreateInputLayouts();
		CreatePipelineState();

		m_frameConstants.Init(m_pDevice.Get());
		m_drawConstants.Init(m_pDevice.Get());
		m_lightConstants.Init(m_pDevice.Get());
		m_materialConstants.Init(m_pDevice.Get());
		m_uiFrameConstants.Init(m_pDevice.Get());

		return RESULT_OK;
	}

	result_t Graphics_Impl::BeginFrame()
	{
		++m_frameTimestamp;

		ClearStateCache();

		FrameConstantData& t(m_frameConstants.Get());
		t.time[0] = (float)System::GetElapsedTime();
		t.time[1] = System::GetDeltaTime();
		t.time[2] = 0.f;
		t.time[3] = 0.f;

		auto surfaceRemoveLambda = [=](const TransientSurface& rSurface) {
			if ((rSurface.m_timestamp + 1) < m_frameTimestamp)
			{
				Resources::ResourceManager<Texture>::Instance().Release(rSurface.m_textureId);
				return true;
			}
			return false;
		};

		m_transientSurfaces.erase(std::remove_if(m_transientSurfaces.begin(), m_transientSurfaces.end(), surfaceRemoveLambda),
								  m_transientSurfaces.end());

		bool bQuit = UpdateMessageLoop();

		m_pDeviceContext->End(m_pGPUBeginFrameTime.Get());
		m_pDeviceContext->Begin(m_pGPUDisjointQuery.Get());

		PushMarker("Generate Mipmaps");
		for (auto pSRV : m_mipQueue)
		{
			m_pDeviceContext->GenerateMips(pSRV);
		}
		PopMarker();

		m_mipQueue.clear();

		f32 fClearColour[] = {0.0f, 0.0f, 0.0f, 1.0f};
		m_pDeviceContext->ClearRenderTargetView(m_pBackBufferRTV.Get(), m_defaultRenderTargetClearColour);

		m_pDeviceContext->ClearDepthStencilView(m_pDefaultDSV.Get(), D3D11_CLEAR_DEPTH, 1.0f, 0);

		ID3D11RenderTargetView* rtvs[] = {m_pBackBufferRTV.Get()};
		m_pDeviceContext->OMSetRenderTargets(1, rtvs, m_pDefaultDSV.Get());

		D3D11_VIEWPORT viewports[]{{0, 0, (f32)m_nSurfaceWidth, (f32)m_nSurfaceHeight, 0.f, 1.0f}};
		m_pDeviceContext->RSSetViewports(1, viewports);

		m_pDeviceContext->RSSetState(m_pFallbackRasterState.Get());

		m_pDeviceContext->OMSetBlendState(m_pBlendStateOpaque.Get(), NULL, 0xffffffff);

		UpdateUITransform();

		m_uiFrameConstants.UpdateGPU(m_pDeviceContext.Get());

		for (IGraphicsCallbacks* pCallbacks : m_graphicsCallbacks)
		{
			pCallbacks->OnBeginFrame();
		}

		if (bQuit)
		{
			return RESULT_QUIT;
		}

		return RESULT_OK;
	}

	result_t Graphics_Impl::EndFrame()
	{
		for (IGraphicsCallbacks* pCallbacks : m_graphicsCallbacks)
		{
			pCallbacks->OnEndFrame();
		}

		m_pDeviceContext->End(m_pGPUEndFrameTime.Get());

		m_pSwapChain->Present(1, 0);

		m_pDeviceContext->End(m_pGPUDisjointQuery.Get());

		D3D11_QUERY_DATA_TIMESTAMP_DISJOINT disjointData;
		while (m_pDeviceContext->GetData(m_pGPUDisjointQuery.Get(), NULL, 0, 0) == S_FALSE)
		{
		}
		m_pDeviceContext->GetData(m_pGPUDisjointQuery.Get(), &disjointData, sizeof(disjointData), 0);
		if (disjointData.Disjoint)
		{
			m_gpuFrameTimeMS = 0.f;
			return RESULT_OK;
		}

		UINT64 beginTS, endTS;
		while (m_pDeviceContext->GetData(m_pGPUBeginFrameTime.Get(), NULL, 0, 0) == S_FALSE)
		{
		}
		m_pDeviceContext->GetData(m_pGPUBeginFrameTime.Get(), &beginTS, sizeof(UINT64), 0);

		while (m_pDeviceContext->GetData(m_pGPUEndFrameTime.Get(), NULL, 0, 0) == S_FALSE)
		{
		}
		m_pDeviceContext->GetData(m_pGPUEndFrameTime.Get(), &endTS, sizeof(UINT64), 0);
		m_gpuFrameTimeMS = float(endTS - beginTS) / float(disjointData.Frequency) * 1000.0f;

		return RESULT_OK;
	}

	void Graphics_Impl::Flush()
	{
		if (m_pDeviceContext)
		{
			m_pDeviceContext->ClearState();
			m_pDeviceContext->Flush();
		}
	}

	ShaderId Graphics_Impl::GetMaterialShader(MaterialShaderKey key)
	{
		if (m_meshShaders[key.m_value].IsInvalid())
		{
			CompileMaterialShader(key);
		}
		return m_meshShaders[key.m_value];
	}

	void Graphics_Impl::SetLightPosition(u32 index, const Vector3f& vPosition)
	{
		m_lightConstants.Get().lightPos[index] = Vector4f(vPosition, 0.f);
	}

	void Graphics_Impl::SetLightDirection(u32 index, const Vector3f& vDirection)
	{
		m_lightConstants.Get().lightDir[index] = Vector4f(normalize(vDirection), 0.f);
	}

	void Graphics_Impl::SetLightColour(u32 index, ColourValue colour)
	{
		colour.as_float_rgba_srgb(&m_lightConstants.Get().lightColour[index].x);
	}

	void Graphics_Impl::SetAmbientLight(ColourValue colour)
	{
		colour.as_float_rgba_srgb(&m_lightConstants.Get().lightAmbient.x);
	}

	void Graphics_Impl::SetDefaultRenderTargetClearColour(ColourValue colour)
	{
		colour.as_float_rgba_srgb(m_defaultRenderTargetClearColour);
	}

	void Graphics_Impl::QueueGenerateMips(ID3D11ShaderResourceView* pSRV)
	{
		m_mipQueue.push_back(pSRV);
	}

	void Graphics_Impl::RegisterWindowCallback(WindowCallback callback)
	{
		m_wndCallbacks.push_back(callback);
	}

	void Graphics_Impl::RegisterGraphicsCallbacks(IGraphicsCallbacks* pCallbacks)
	{
		PLAY_ASSERT(pCallbacks);
		m_graphicsCallbacks.push_back(pCallbacks);
	}

	result_t Graphics_Impl::LoadImageAsRGBA(const char* pFilePath, std::vector<u8>& imageOut, SurfaceSize& surfaceSizeOut)
	{
		if (!System::CheckFileExists(pFilePath))
		{
			Debug::Printf("ERROR: Image file does not exist! path='%s'\n", pFilePath);
			return Result::RESULT_FAIL;
		}

		ComPtr<IWICImagingFactory> pIWICFactory;
		ComPtr<IWICBitmapDecoder> pDecoder;
		ComPtr<IWICFormatConverter> pConvertedSourceBitmap;
		ComPtr<IWICBitmapFrameDecode> pFrame;

		HRESULT hr;

		if (!pIWICFactory)
		{
			hr = CoCreateInstance(CLSID_WICImagingFactory, NULL, CLSCTX_INPROC_SERVER, IID_PPV_ARGS(pIWICFactory.GetAddressOf()));

			PLAY_ASSERT(SUCCEEDED(hr));
		}

		size_t fileSizeBytes = 0;
		const void* pFileData = System::LoadFileData(pFilePath, fileSizeBytes);
		if (!pFileData)
		{
			Debug::Printf("ERROR: Image file load data failed! path='%s'\n", pFilePath);
			return Result::RESULT_FAIL;
		}

		HGLOBAL hMem = ::GlobalAlloc(GMEM_MOVEABLE, fileSizeBytes);
		if (!hMem)
		{
			Debug::Printf("ERROR: Failed to allocate HGLOBAL memory for image path='%s'\n", pFilePath);
			return Result::RESULT_FAIL;
		}

		{
			LPVOID pDest = ::GlobalLock(hMem);
			memcpy(pDest, pFileData, fileSizeBytes);
			::GlobalUnlock(hMem);
		}

		ComPtr<IStream> pStream;
		hr = CreateStreamOnHGlobal(hMem, TRUE, &pStream);
		if (FAILED(hr))
		{
			GlobalFree(hMem);
			Debug::Printf("ERROR: Failed to create IStream for image path='%s'\n", pFilePath);
			return Result::RESULT_FAIL;
		}

		hr = pIWICFactory->CreateDecoderFromStream(pStream.Get(), NULL, WICDecodeMetadataCacheOnDemand, pDecoder.GetAddressOf());

		if (SUCCEEDED(hr))
		{
			hr = pDecoder->GetFrame(0, &pFrame);
		}

		if (SUCCEEDED(hr))
		{
			hr = pIWICFactory->CreateFormatConverter(pConvertedSourceBitmap.GetAddressOf());
		}

		if (SUCCEEDED(hr))
		{
			hr = pConvertedSourceBitmap->Initialize(pFrame.Get(),
													GUID_WICPixelFormat32bppRGBA,
													WICBitmapDitherTypeNone,
													NULL,
													0.f,
													WICBitmapPaletteTypeCustom);
		}

		if (SUCCEEDED(hr))
		{
			pConvertedSourceBitmap->GetSize(&surfaceSizeOut.m_width, &surfaceSizeOut.m_height);

			u32 sizeBytes = surfaceSizeOut.m_width * surfaceSizeOut.m_height * 4;
			imageOut.resize(sizeBytes);

			hr = pConvertedSourceBitmap->CopyPixels(NULL, surfaceSizeOut.m_width * sizeof(u32), sizeBytes, imageOut.data());
			if (SUCCEEDED(hr))
			{
				return Result::RESULT_OK;
			}
		}

		return Result::RESULT_FAIL;
	}

	void Graphics_Impl::BindComputeShader(ShaderId shaderId)
	{
		ID3D11DeviceContext* pDC = m_pDeviceContext.Get();
		Shader* pCS = Resources::ResourceManager<Shader>::Instance().GetPtr(shaderId);
		pCS->Bind(pDC);
	}

	void Graphics_Impl::Dispatch(u32 groupCountX, u32 groupCountY, u32 groupCountZ)
	{
		ID3D11DeviceContext* pDC = m_pDeviceContext.Get();
		pDC->Dispatch(groupCountX, groupCountY, groupCountZ);
	}

	void Graphics_Impl::InternalBindMaterial(Material* pMaterial, ID3D11DeviceContext* pDC)
	{
		if (pMaterial)
		{
			bool bRequiresBind = false;
			if (pMaterial->m_bModifiedFlag)
			{
				pMaterial->SetModified(false);
				bRequiresBind = true;
			}
			if (m_pPrevMaterial != pMaterial)
			{
				m_pPrevMaterial = pMaterial;
				bRequiresBind = true;
			}
			if (bRequiresBind)
			{
				pDC->RSSetState(pMaterial->m_pRasterState.Get());
				pDC->OMSetDepthStencilState(pMaterial->m_pDepthStencilState.Get(), 0);
				pDC->OMSetBlendState(pMaterial->m_pBlendState.Get(), NULL, 0xffffffff);

				Shader* pVS = Resources::ResourceManager<Shader>::Instance().GetPtr(pMaterial->m_VertexShader);
				if (!pVS)
				{
					pVS = Resources::ResourceManager<Shader>::Instance().GetPtr(m_meshShaders[0]);
				}
				pVS->Bind(pDC);

				if (!pMaterial->m_bNullPixelShader)
				{
					Shader* pPS = Resources::ResourceManager<Shader>::Instance().GetPtr(pMaterial->m_PixelShader);
					if (!pPS)
					{
						pPS = Resources::ResourceManager<Shader>::Instance().GetPtr(m_meshShaders[1]);
					}
					pPS->Bind(pDC);
				}
				else
				{
					pDC->PSSetShader(nullptr, nullptr, 0);
				}

				ID3D11Buffer* buffers[] = {pMaterial->m_pMaterialConstants.Get()};
				pDC->VSSetConstantBuffers(3, 1, buffers);

				if (!pMaterial->m_bNullPixelShader)
				{
					pDC->PSSetConstantBuffers(3, 1, buffers);

					ID3D11ShaderResourceView* textureBindings[kMaxMaterialTextureSlots];
					ID3D11SamplerState* samplerBindings[kMaxMaterialTextureSlots];

					for (u32 i = 0; i < kMaxMaterialTextureSlots; ++i)
					{
						if (pMaterial->m_texture[i].IsValid())
						{
							Texture* pTexture = Resources::ResourceManager<Texture>::Instance().GetPtr(pMaterial->m_texture[i]);
							Sampler* pSampler = Resources::ResourceManager<Sampler>::Instance().GetPtr(pMaterial->m_sampler[i]);

							textureBindings[i] = pTexture ? pTexture->m_pSRV.Get() : nullptr;
							samplerBindings[i] = pSampler ? pSampler->m_pSampler.Get() : nullptr;
						}
						else
						{
							textureBindings[i] = nullptr;
							samplerBindings[i] = nullptr;
						}
					}
					pDC->PSSetShaderResources(0, 4, textureBindings);
					pDC->PSSetSamplers(0, 4, samplerBindings);
				}
			}
		}
		else
		{
			pDC->RSSetState(m_pFallbackRasterState.Get());
			pDC->OMSetDepthStencilState(m_pFallbackDepthStencilState.Get(), 0);
			pDC->OMSetBlendState(m_pBlendStateOpaque.Get(), NULL, 0xffffffff);
			Shader* pVS = Resources::ResourceManager<Shader>::Instance().GetPtr(m_meshShaders[0]);
			Shader* pPS = Resources::ResourceManager<Shader>::Instance().GetPtr(m_meshShaders[1]);
			pVS->Bind(pDC);
			pPS->Bind(pDC);

			m_materialConstants.Bind(pDC, 3);

			m_pPrevMaterial = nullptr;
		}
	}

	void Graphics_Impl::InternalBindMesh(const Mesh* pMesh, ID3D11DeviceContext* pDC)
	{
		if (pMesh != m_pPrevMesh)
		{
			m_pPrevMesh = pMesh;

			if (pMesh->m_bIsSkinnedMesh)
			{
				pDC->IASetInputLayout(m_pSkinnedMeshInputLayout.Get());
			}
			else
			{
				pDC->IASetInputLayout(m_pMeshInputLayout.Get());
			}

			pMesh->Bind(pDC);
		}
	}

	void Graphics_Impl::DrawMesh(const Mesh* pMesh, u32 elementOffset, u32 elementCount)
	{
		PLAY_ASSERT(pMesh);

		UpdateConstantBuffers();

		ID3D11DeviceContext* pDC = m_pDeviceContext.Get();

		BindActiveMaterial(pDC);

		InternalBindMesh(pMesh, pDC);

		m_frameConstants.Bind(pDC, 0);
		m_drawConstants.Bind(pDC, 1);
		m_lightConstants.Bind(pDC, 2);

		if (pMesh->m_pIndexBuffer)
		{
			pDC->DrawIndexed(elementCount != ~0u ? elementCount : pMesh->m_indexCount, elementOffset, 0);
		}
		else
		{
			pDC->Draw(elementCount != ~0u ? elementCount : pMesh->m_vertexCount, elementOffset);
		}
	}

	void Graphics_Impl::DrawInstancedMesh(const Mesh* pMesh, u32 kInstanceCount, u32 kInstanceOffset, u32 elementOffset, u32 elementCount)
	{
		PLAY_ASSERT(pMesh);

		UpdateConstantBuffers();

		ID3D11DeviceContext* pDC = m_pDeviceContext.Get();

		BindActiveMaterial(pDC);

		InternalBindMesh(pMesh, pDC);

		m_frameConstants.Bind(pDC, 0);
		m_drawConstants.Bind(pDC, 1);
		m_lightConstants.Bind(pDC, 2);

		if (pMesh->m_pIndexBuffer)
		{
			pDC->DrawIndexedInstanced(elementCount != ~0u ? elementCount : pMesh->m_indexCount,
									  kInstanceCount,
									  elementOffset,
									  0,
									  kInstanceOffset);
		}
		else
		{
			pDC->DrawInstanced(elementCount != ~0u ? elementCount : pMesh->m_vertexCount, kInstanceCount, elementOffset, kInstanceOffset);
		}
	}

	void Graphics_Impl::DrawWithoutVertices(u32 elements)
	{
		UpdateConstantBuffers();

		ID3D11DeviceContext* pDC = m_pDeviceContext.Get();

		BindActiveMaterial(pDC);

		m_pPrevMesh = nullptr;

		pDC->IASetInputLayout(nullptr);

		m_frameConstants.Bind(pDC, 0);
		m_drawConstants.Bind(pDC, 1);
		m_lightConstants.Bind(pDC, 2);

		pDC->IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
		pDC->Draw(elements, 0);
	}

	void Graphics_Impl::DrawIndirectWithoutVertices(BufferId hArgBuffer, u32 offset)
	{
		Buffer* pBuffer = Resources::ResourceManager<Buffer>::Instance().GetPtr(hArgBuffer);
		PLAY_ASSERT(pBuffer);
		if (pBuffer)
		{
			UpdateConstantBuffers();

			ID3D11DeviceContext* pDC = m_pDeviceContext.Get();

			BindActiveMaterial(pDC);

			m_pPrevMesh = nullptr;

			pDC->IASetInputLayout(nullptr);

			m_frameConstants.Bind(pDC, 0);
			m_drawConstants.Bind(pDC, 1);
			m_lightConstants.Bind(pDC, 2);

			pDC->IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST);

			pDC->DrawInstancedIndirect(pBuffer->GetBuffer(), offset);
		}
	}

	void Graphics_Impl::SetViewport(const Viewport& v)
	{
		D3D11_VIEWPORT viewports[]{{0, 0, (f32)m_nSurfaceWidth, (f32)m_nSurfaceHeight, 0.f, 1.0f}};
		m_pDeviceContext->RSSetViewports(1, (const D3D11_VIEWPORT*)&v);
	}

	void Graphics_Impl::SetViewMatrix(const Matrix4x4f& m)
	{
		FrameConstantData& t(m_frameConstants.Get());
		t.viewMtx = m;
		t.viewProjectionMtx = t.projectionMtx * t.viewMtx;
		t.invViewMtx = AffineInverse(t.viewMtx);
		t.viewPosition = t.invViewMtx.m_column[3];
	}

	void Graphics_Impl::SetProjectionMatrix(const Matrix4x4f& m)
	{
		FrameConstantData& t(m_frameConstants.Get());
		t.projectionMtx = m;
		t.viewProjectionMtx = t.projectionMtx * t.viewMtx;
		t.invProjectionMtx = InversePerspectiveProjectRH(t.projectionMtx);
	}

	void Graphics_Impl::SetWorldMatrix(const Matrix4x4f& m)
	{
		DrawConstantData& t(m_drawConstants.Get());
		t.worldMtx = m;
		t.normalMtx = Matrix4x4f(m.Upper3x3(), Vector3f(0, 0, 0));
		t.mvpMtx = m_frameConstants.Get().viewProjectionMtx * m;
	}

	void Graphics_Impl::UpdateUITransform()
	{
		m_uiFrameConstants.Get().viewProjectionMtx =
			MatrixOrthoProjectRH(0.f, (f32)m_nRenderTargetWidth, (f32)m_nRenderTargetHeight, 0.f, 0.f, 1.f);
		m_uiFrameConstants.Get().viewportRect = Vector4f(0.f, 0.f, (f32)m_nRenderTargetWidth, (f32)m_nRenderTargetHeight);
	}

	void Graphics_Impl::SetMaterial(MaterialId materialId)
	{
		m_activeMaterial = materialId;
	}

	void Graphics_Impl::UpdateMaterialConstants(MaterialId materialId, const void* pData, size_t size)
	{
		Material* pMaterial = Resources::ResourceManager<Material>::Instance().GetPtr(materialId);
		if (pMaterial)
		{
			auto* pBuffer(pMaterial->m_pMaterialConstants.Get());
			D3D11_MAPPED_SUBRESOURCE data;
			HRESULT hr = m_pDeviceContext->Map(pBuffer, 0, D3D11_MAP::D3D11_MAP_WRITE_DISCARD, 0, &data);
			if (SUCCEEDED(hr))
			{
				memcpy(data.pData, pData, size);
				m_pDeviceContext->Unmap(pBuffer, 0);
			}
		}
	}

	void Graphics_Impl::UpdateBuffer(BufferId bufferId, const void* pData, size_t size)
	{
		Buffer* pBuffer = Resources::ResourceManager<Buffer>::Instance().GetPtr(bufferId);
		if (pBuffer)
		{
			D3D11_MAPPED_SUBRESOURCE data;
			HRESULT hr = m_pDeviceContext->Map(pBuffer->GetBuffer(), 0, D3D11_MAP::D3D11_MAP_WRITE_DISCARD, 0, &data);
			if (SUCCEEDED(hr))
			{
				memcpy(data.pData, pData, size);
				m_pDeviceContext->Unmap(pBuffer->GetBuffer(), 0);
			}
		}
	}

	void Graphics_Impl::UpdateTexture(TextureId textureId, u32 mipId, u32 sliceId, const void* pData, size_t rowPitchBytes)
	{
		Texture* pTexture = Resources::ResourceManager<Texture>::Instance().GetPtr(textureId);
		if (pTexture)
		{
			D3D11_TEXTURE2D_DESC desc;
			ID3D11Texture2D* pD3DTexture = pTexture->GetTexture();
			PLAY_ASSERT(pD3DTexture);

			pD3DTexture->GetDesc(&desc);

			UINT subResource = sliceId * desc.MipLevels + mipId;

			D3D11_MAPPED_SUBRESOURCE data;
			HRESULT hr = m_pDeviceContext->Map(pD3DTexture, subResource, D3D11_MAP::D3D11_MAP_WRITE_DISCARD, 0, &data);

			PLAY_ASSERT((size_t)data.RowPitch >= rowPitchBytes);

			if (SUCCEEDED(hr))
			{
				if ((size_t)data.RowPitch == rowPitchBytes)
				{
					memcpy(data.pData, pData, rowPitchBytes * desc.Height);
				}
				else
				{
					PLAY_ASSERT((size_t)data.RowPitch >= rowPitchBytes);

					const char* pSrcRow = static_cast<const char*>(pData);
					char* pDstRow = static_cast<char*>(data.pData);

					for (u32 row = 0; row < desc.Height; ++row)
					{
						memcpy(pDstRow, pSrcRow, rowPitchBytes);

						pSrcRow += rowPitchBytes;
						pDstRow += data.RowPitch;
					}
				}
				m_pDeviceContext->Unmap(pD3DTexture, 0);
			}
		}
	}

	void Graphics_Impl::BindGlobalTexture(u32 slot, TextureId textureId, SamplerId samplerId, ShaderStageFlag::Type stageBinding)
	{
		Texture* pTexture = Resources::ResourceManager<Texture>::Instance().GetPtr(textureId);
		Sampler* pSampler = Resources::ResourceManager<Sampler>::Instance().GetPtr(samplerId);

		ID3D11ShaderResourceView* pSRV[] = {pTexture ? pTexture->m_pSRV.Get() : nullptr};
		ID3D11SamplerState* pSamplerState[] = {pSampler ? pSampler->m_pSampler.Get() : nullptr};

		if (stageBinding & ShaderStageFlag::VERTEX_STAGE)
		{
			m_pDeviceContext->VSSetShaderResources(slot, 1, pSRV);
			m_pDeviceContext->VSSetSamplers(slot, 1, pSamplerState);
		}

		if (stageBinding & ShaderStageFlag::PIXEL_STAGE)
		{
			m_pDeviceContext->PSSetShaderResources(slot, 1, pSRV);
			m_pDeviceContext->PSSetSamplers(slot, 1, pSamplerState);
		}

		if (stageBinding & ShaderStageFlag::COMPUTE_STAGE)
		{
			m_pDeviceContext->CSSetShaderResources(slot, 1, pSRV);
			m_pDeviceContext->CSSetSamplers(slot, 1, pSamplerState);
		}
	}

	void Graphics_Impl::BindGlobalBufferCBV(u32 slot, BufferId bufferId, ShaderStageFlag::Type stageBinding)
	{
		Buffer* pBuffer = Resources::ResourceManager<Buffer>::Instance().GetPtr(bufferId);
		ID3D11Buffer* pBufferViews[] = {pBuffer ? pBuffer->GetBuffer() : nullptr};

		if (stageBinding & ShaderStageFlag::VERTEX_STAGE)
		{
			m_pDeviceContext->VSSetConstantBuffers(slot, 1, pBufferViews);
		}

		if (stageBinding & ShaderStageFlag::PIXEL_STAGE)
		{
			m_pDeviceContext->PSSetConstantBuffers(slot, 1, pBufferViews);
		}

		if (stageBinding & ShaderStageFlag::COMPUTE_STAGE)
		{
			m_pDeviceContext->CSSetConstantBuffers(slot, 1, pBufferViews);
		}
	}

	void Graphics_Impl::BindGlobalBufferSRV(u32 slot, BufferId bufferId, ShaderStageFlag::Type stageBinding)
	{
		Buffer* pBuffer = Resources::ResourceManager<Buffer>::Instance().GetPtr(bufferId);
		ID3D11ShaderResourceView* pBufferViews[] = {pBuffer ? pBuffer->GetSRV() : nullptr};

		if (stageBinding & ShaderStageFlag::VERTEX_STAGE)
		{
			m_pDeviceContext->VSSetShaderResources(slot, 1, pBufferViews);
		}

		if (stageBinding & ShaderStageFlag::PIXEL_STAGE)
		{
			m_pDeviceContext->PSSetShaderResources(slot, 1, pBufferViews);
		}

		if (stageBinding & ShaderStageFlag::COMPUTE_STAGE)
		{
			m_pDeviceContext->CSSetShaderResources(slot, 1, pBufferViews);
		}
	}

	void Graphics_Impl::BindGlobalBufferUAV(u32 slot, BufferId bufferId, ShaderStageFlag::Type stageBinding)
	{
		Buffer* pBuffer = Resources::ResourceManager<Buffer>::Instance().GetPtr(bufferId);
		ID3D11UnorderedAccessView* pBufferViews[] = {pBuffer ? pBuffer->GetUAV() : nullptr};

		if (stageBinding & ShaderStageFlag::COMPUTE_STAGE)
		{
			m_pDeviceContext->CSSetUnorderedAccessViews(slot, 1, pBufferViews, NULL);
		}
	}

	TextureId Graphics_Impl::GetTransientSurface(SurfaceSize surfaceSize, TextureFormat format)
	{
		SurfaceKey key(surfaceSize, format);

		for (auto& rSurfaceInfo : m_transientSurfaces)
		{
			if (rSurfaceInfo.m_key == key && rSurfaceInfo.m_timestamp != m_frameTimestamp)
			{
				rSurfaceInfo.m_timestamp = m_frameTimestamp;
				return rSurfaceInfo.m_textureId;
			}
		}
		TransientSurface info;
		info.m_key = key;
		info.m_timestamp = m_frameTimestamp;

		TextureDesc desc = {};
		desc.m_width = surfaceSize.m_width;
		desc.m_height = surfaceSize.m_height;
		desc.m_format = format;
		desc.m_flags = TextureFlags::ENABLE_TEXTURE | TextureFlags::ENABLE_RENDER_TARGET;
		desc.m_pImageData = nullptr;

		info.m_textureId = Resources::CreateAsset<Texture>(desc);

		m_transientSurfaces.push_back(info);

		return info.m_textureId;
	}

	SurfaceSize Graphics_Impl::GetDisplaySurfaceSize() const
	{
		return {m_nSurfaceWidth, m_nSurfaceHeight};
	}

	SurfaceSize Graphics_Impl::GetRenderTargetSize() const
	{
		return {m_nRenderTargetWidth, m_nRenderTargetHeight};
	}

	void Graphics_Impl::SetRenderTargets(const TextureId* colourTargetIdArray, u32 mrtCount, TextureId depthTextureId)
	{
		PLAY_ASSERT(mrtCount <= D3D11_SIMULTANEOUS_RENDER_TARGET_COUNT);

		ID3D11RenderTargetView* rtvs[D3D11_SIMULTANEOUS_RENDER_TARGET_COUNT];
		ID3D11DepthStencilView* pDSV = nullptr;

		m_nRenderTargetWidth = m_nSurfaceWidth;
		m_nRenderTargetHeight = m_nSurfaceHeight;

		for (u32 i = 0; i < mrtCount; ++i)
		{
			Texture* pRenderTarget = Resources::ResourceManager<Texture>::Instance().GetPtr(colourTargetIdArray[i]);
			if (pRenderTarget)
			{
				rtvs[i] = pRenderTarget->m_pRTV.Get();

				D3D11_TEXTURE2D_DESC desc;
				pRenderTarget->GetTexture()->GetDesc(&desc);
				m_nRenderTargetWidth = desc.Width;
				m_nRenderTargetHeight = desc.Height;
			}
			else
			{
				rtvs[i] = nullptr;
			}
		}

		Texture* pDepthTarget = Resources::ResourceManager<Texture>::Instance().GetPtr(depthTextureId);
		if (pDepthTarget)
		{
			pDSV = pDepthTarget->m_pDSV.Get();

			D3D11_TEXTURE2D_DESC desc;
			pDepthTarget->GetTexture()->GetDesc(&desc);
			m_nRenderTargetWidth = desc.Width;
			m_nRenderTargetHeight = desc.Height;
		}

		m_pDeviceContext->OMSetRenderTargets(mrtCount, rtvs, pDSV);

		UpdateUITransform();
	}

	void Graphics_Impl::SetRenderTargetsToSwapChain(bool bEnableDefaultDepth)
	{
		ID3D11RenderTargetView* rtvs[] = {m_pBackBufferRTV.Get()};
		ID3D11DepthStencilView* pDSV = bEnableDefaultDepth ? m_pDefaultDSV.Get() : nullptr;
		m_pDeviceContext->OMSetRenderTargets(1, rtvs, pDSV);

		m_nRenderTargetWidth = m_nSurfaceWidth;
		m_nRenderTargetHeight = m_nSurfaceHeight;
		UpdateUITransform();
	}

	void Graphics_Impl::SetRenderTargetsToSwapChain(TextureId depthTextureId)
	{
		ID3D11RenderTargetView* rtvs[] = {m_pBackBufferRTV.Get()};
		ID3D11DepthStencilView* pDSV = nullptr;

		Texture* pDepthTarget = Resources::ResourceManager<Texture>::Instance().GetPtr(depthTextureId);
		if (pDepthTarget)
		{
			pDSV = pDepthTarget->m_pDSV.Get();
		}
		m_pDeviceContext->OMSetRenderTargets(1, rtvs, pDSV);

		m_nRenderTargetWidth = m_nSurfaceWidth;
		m_nRenderTargetHeight = m_nSurfaceHeight;
		UpdateUITransform();
	}

	void Graphics_Impl::ClearDepthTarget(TextureId textureId, f32 depthValue)
	{
		if (textureId.IsValid())
		{
			Texture* pTarget = Resources::ResourceManager<Texture>::Instance().GetPtr(textureId);
			if (pTarget && pTarget->m_pDSV)
			{
				m_pDeviceContext->ClearDepthStencilView(pTarget->m_pDSV.Get(), D3D11_CLEAR_DEPTH, depthValue, 0);
			}
		}
		else
		{
			m_pDeviceContext->ClearDepthStencilView(m_pDefaultDSV.Get(), D3D11_CLEAR_DEPTH, depthValue, 0);
		}
	}

	void Graphics_Impl::ClearRenderTarget(TextureId textureId, ColourValue clearColour)
	{
		f32 fClearColour[4];
		clearColour.as_float_rgba_srgb(fClearColour);

		if (textureId.IsValid())
		{
			Texture* pTarget = Resources::ResourceManager<Texture>::Instance().GetPtr(textureId);
			if (pTarget && pTarget->m_pRTV)
			{
				m_pDeviceContext->ClearRenderTargetView(pTarget->m_pRTV.Get(), fClearColour);
			}
		}
		else
		{
			m_pDeviceContext->ClearRenderTargetView(m_pBackBufferRTV.Get(), fClearColour);
		}
	}

	result_t Graphics_Impl::InitWindow(const SystemDesc& rDesc)
	{
		SetProcessDpiAwarenessContext(DPI_AWARENESS_CONTEXT_PER_MONITOR_AWARE);

		u32 width = rDesc.width;
		u32 height = rDesc.height;
		const wchar_t* wndClassName = L"Play3d";

		WNDCLASSEXW wcex;

		wcex.cbSize = sizeof(WNDCLASSEXW);

		wcex.style = CS_HREDRAW | CS_VREDRAW;
		wcex.lpfnWndProc = Graphics_Impl::MainWndProc;
		wcex.cbClsExtra = 0;
		wcex.cbWndExtra = 0;
		wcex.hInstance = m_hInstance;
		wcex.hIcon = LoadIcon(m_hInstance, IDI_APPLICATION);
		wcex.hCursor = LoadCursor(nullptr, IDC_ARROW);
		wcex.hbrBackground = (HBRUSH)(COLOR_WINDOW + 1);
		wcex.lpszMenuName = nullptr;
		wcex.lpszClassName = wndClassName;
		wcex.hIconSm = LoadIcon(wcex.hInstance, IDI_APPLICATION);

		RegisterClassExW(&wcex);

		UINT dwStyle = WS_VISIBLE | WS_OVERLAPPED | WS_CAPTION | WS_SYSMENU;
		RECT rect = {0, 0, (LONG)width, (LONG)height};
		AdjustWindowRect(&rect, dwStyle, FALSE);
		m_hWnd = CreateWindowW(wndClassName,
							   wndClassName,
							   dwStyle,
							   CW_USEDEFAULT,
							   CW_USEDEFAULT,
							   rect.right - rect.left,
							   rect.bottom - rect.top,
							   nullptr,
							   nullptr,
							   m_hInstance,
							   this);
		PLAY_ASSERT_MSG(m_hWnd != NULL, "Could not create the application window.");

		if (!m_hWnd)
		{
			return RESULT_FAIL;
		}
		SetWindowTextA(m_hWnd, rDesc.title);
		ShowWindow(m_hWnd, SW_SHOWNORMAL);
		UpdateWindow(m_hWnd);

		m_nSurfaceWidth = width;
		m_nSurfaceHeight = height;
		m_nRenderTargetWidth = width;
		m_nRenderTargetHeight = height;

		return RESULT_OK;
	}

	result_t Graphics_Impl::ReleaseWindow()
	{
		if (m_hWnd)
		{
			DestroyWindow(m_hWnd);
			m_hWnd = NULL;
			return RESULT_OK;
		}
		return RESULT_FAIL;
	}

	result_t Graphics_Impl::InitDirectX()
	{

		HRESULT hr;

		DWORD createDeviceFlags = 0;
#ifdef _DEBUG
		createDeviceFlags |= D3D11_CREATE_DEVICE_DEBUG;
#endif

		D3D_FEATURE_LEVEL featureLevel;
		hr = D3D11CreateDevice(nullptr,
							   D3D_DRIVER_TYPE_HARDWARE,
							   nullptr,
							   createDeviceFlags,
							   nullptr,
							   0,
							   D3D11_SDK_VERSION,
							   &m_pDevice,
							   &featureLevel,
							   &m_pDeviceContext);
		if (FAILED(hr))
		{
			return RESULT_FAIL;
		}

		hr = m_pDeviceContext.As(&m_pAnnotation);
		if (FAILED(hr))
		{
			return RESULT_FAIL;
		}

		hr = m_pDevice.As(&m_pDXGIDevice);
		if (FAILED(hr))
		{
			return RESULT_FAIL;
		}

		hr = m_pDXGIDevice->GetAdapter(&m_pDXGIAdapter);
		if (FAILED(hr))
		{
			return RESULT_FAIL;
		}

		hr = m_pDXGIAdapter->GetParent(IID_PPV_ARGS(&m_pDXGIFactory));
		if (FAILED(hr))
		{
			return RESULT_FAIL;
		}

		DXGI_SWAP_CHAIN_DESC1 swapDesc = {};
		swapDesc.Width = m_nSurfaceWidth;
		swapDesc.Height = m_nSurfaceHeight;
		swapDesc.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
		swapDesc.Stereo = FALSE;
		swapDesc.SampleDesc.Count = 1;
		swapDesc.SampleDesc.Quality = 0;
		swapDesc.BufferUsage = DXGI_USAGE_RENDER_TARGET_OUTPUT;
		swapDesc.BufferCount = 2;
		swapDesc.Scaling = DXGI_SCALING_STRETCH;
		swapDesc.SwapEffect = DXGI_SWAP_EFFECT_FLIP_DISCARD;
		swapDesc.AlphaMode = DXGI_ALPHA_MODE_IGNORE;
		swapDesc.Flags = 0;

		hr = m_pDXGIFactory->CreateSwapChainForHwnd(m_pDevice.Get(), m_hWnd, &swapDesc, NULL, NULL, &m_pSwapChain);
		if (FAILED(hr))
		{
			return RESULT_FAIL;
		}

		result_t result = RecreateRenderTargetView();
		if (result != RESULT_OK)
		{
			return result;
		}

		result = RecreateDepthBuffer();
		if (result != RESULT_OK)
		{
			return result;
		}

		result = InitTimestamps();
		if (result != RESULT_OK)
		{
			return result;
		}

		return RESULT_OK;
	}

	result_t Graphics_Impl::InitTimestamps()
	{
		HRESULT hr;
		D3D11_QUERY_DESC timeStampDesc = {};
		timeStampDesc.Query = D3D11_QUERY_TIMESTAMP;
		hr = m_pDevice->CreateQuery(&timeStampDesc, &m_pGPUBeginFrameTime);
		if (FAILED(hr))
		{
			return RESULT_FAIL;
		}
		hr = m_pDevice->CreateQuery(&timeStampDesc, &m_pGPUEndFrameTime);
		if (FAILED(hr))
		{
			return RESULT_FAIL;
		}

		D3D11_QUERY_DESC disjointDesc = {};
		disjointDesc.Query = D3D11_QUERY_TIMESTAMP_DISJOINT;
		hr = m_pDevice->CreateQuery(&disjointDesc, &m_pGPUDisjointQuery);
		if (FAILED(hr))
		{
			return RESULT_FAIL;
		}
		return RESULT_OK;
	}

	result_t Graphics_Impl::Resize(u32 width, u32 height)
	{
		if (!(m_pDevice && m_pDeviceContext && m_pSwapChain))
		{
			return RESULT_FAIL;
		}

		HRESULT hr;

		m_nSurfaceWidth = width;
		m_nSurfaceHeight = height;
		m_nRenderTargetWidth = width;
		m_nRenderTargetHeight = height;

		m_pDeviceContext->OMSetRenderTargets(0, NULL, NULL);

		m_pDeviceContext->Flush();

		m_pBackBufferRTV.Reset();

		hr = m_pSwapChain->ResizeBuffers(0, 0, 0, DXGI_FORMAT_UNKNOWN, 0);
		if (FAILED(hr))
		{
			return RESULT_FAIL;
		}

		result_t result = RecreateRenderTargetView();
		if (result != RESULT_OK)
		{
			return result;
		}

		result = RecreateDepthBuffer();
		if (result != RESULT_OK)
		{
			return result;
		}

		UpdateUITransform();

		return RESULT_OK;
	}

	result_t Graphics_Impl::RecreateRenderTargetView()
	{
		ComPtr<ID3D11Texture2D> pBackBufferTexture;
		HRESULT hr = m_pSwapChain->GetBuffer(0, IID_PPV_ARGS(&pBackBufferTexture));
		if (FAILED(hr))
		{
			return RESULT_FAIL;
		}

		D3D11_RENDER_TARGET_VIEW_DESC viewDesc = {};
		viewDesc.Format = DXGI_FORMAT_R8G8B8A8_UNORM_SRGB;
		viewDesc.ViewDimension = D3D11_RTV_DIMENSION_TEXTURE2D;
		viewDesc.Texture2D.MipSlice = 0;
		hr = m_pDevice->CreateRenderTargetView(pBackBufferTexture.Get(), &viewDesc, &m_pBackBufferRTV);
		if (FAILED(hr))
		{
			return RESULT_FAIL;
		}
		return RESULT_OK;
	}

	result_t Graphics_Impl::RecreateDepthBuffer()
	{
		m_pDefaultDepthBuffer.Reset();
		m_pDefaultDSV.Reset();

		D3D11_TEXTURE2D_DESC desc = {};
		desc.Width = m_nSurfaceWidth;
		desc.Height = m_nSurfaceHeight;
		desc.MipLevels = 1;
		desc.ArraySize = 1;
		desc.Format = DXGI_FORMAT_D32_FLOAT;
		desc.SampleDesc.Count = 1;
		desc.SampleDesc.Quality = 0;
		desc.Usage = D3D11_USAGE_DEFAULT;
		desc.BindFlags = D3D11_BIND_DEPTH_STENCIL;
		desc.CPUAccessFlags = 0;
		desc.MiscFlags = 0;

		HRESULT hr = m_pDevice->CreateTexture2D(&desc, NULL, &m_pDefaultDepthBuffer);
		if (FAILED(hr))
		{
			return RESULT_FAIL;
		}

		hr = m_pDevice->CreateDepthStencilView(m_pDefaultDepthBuffer.Get(), NULL, &m_pDefaultDSV);
		if (FAILED(hr))
		{
			return RESULT_FAIL;
		}

		return RESULT_OK;
	}

	result_t Graphics_Impl::CompileInternalShaders()
	{
		u32 compilationFlags = 0;
#ifdef _DEBUG
		compilationFlags |= ShaderCompilationFlags::DEBUG;
#endif

		{
			ShaderCompilerDesc desc;
			desc.m_name = "HLSL_SkinnedMeshShader";
			desc.m_hlslCode = HLSL_SkinnedMeshShader;
			desc.m_type = ShaderType::VERTEX_SHADER;
			desc.m_entryPoint = "VS_Main";
			desc.m_flags = compilationFlags;

			m_skinnedMeshVS = Shader::Compile(desc);
		}

		{
			MaterialShaderKey key;
			key.m_value = 0;
			CompileMaterialShader(key);
			key.m_bits.m_pixelShader = 1;
			CompileMaterialShader(key);
		}

		return RESULT_OK;
	}

	void Graphics_Impl::CompileMaterialShader(MaterialShaderKey& key)
	{
		PLAY_ASSERT_MSG(m_meshShaders[key.m_value].IsInvalid(), "Duplicate shader permutation");
		ShaderCompilerDesc desc;
		desc.m_hlslCode = HLSL_MeshShader;
		desc.m_name = "MeshUber";
		desc.m_defines.clear();
		desc.m_flags = 0;
#ifdef _DEBUG
		desc.m_flags |= ShaderCompilationFlags::DEBUG;
#endif

		if (key.m_bits.m_pixelShader)
		{
			desc.m_type = ShaderType::PIXEL_SHADER;
			desc.m_entryPoint = "PS_Main";
			desc.m_name += "_PS";
		}
		else
		{
			desc.m_type = ShaderType::VERTEX_SHADER;
			desc.m_entryPoint = "VS_Main";
			desc.m_name += "_VS";
		}

		if (key.m_bits.m_useTexture0)
		{
			desc.m_defines.push_back({"USE_TEXTURE_0", "1"});
			desc.m_name += "_T0";
		}

		if (key.m_bits.m_useNormalMap)
		{
			desc.m_defines.push_back({"USE_NORMAL_TEXTURE", "1"});
			desc.m_name += "_NM";
		}

		if (key.m_bits.m_useLighting)
		{
			desc.m_defines.push_back({"USE_LIGHTING", "1"});
			desc.m_name += "_L";
		}

		char t[16];
		sprintf_s(t, 16, "%u", key.m_bits.m_lightCount);
		desc.m_defines.push_back({"LIGHT_COUNT", t});
		desc.m_name += "_LC";
		desc.m_name += t;

		m_meshShaders[key.m_value] = Shader::Compile(desc);
	}

	result_t Graphics_Impl::CreateInputLayouts()
	{
		{
			Shader* pVS = Resources::ResourceManager<Shader>::Instance().GetPtr(m_meshShaders[0]);

				D3D11_INPUT_ELEMENT_DESC vertexFormat[] = {
					{"POSITION", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 0, D3D11_INPUT_PER_VERTEX_DATA, 0},
					{"COLOUR", 0, DXGI_FORMAT_R8G8B8A8_UNORM, 1, 0, D3D11_INPUT_PER_VERTEX_DATA, 0},
					{"NORMAL", 0, DXGI_FORMAT_R32G32B32_FLOAT, 2, 0, D3D11_INPUT_PER_VERTEX_DATA, 0},
					{"UV", 0, DXGI_FORMAT_R32G32_FLOAT, 3, 0, D3D11_INPUT_PER_VERTEX_DATA, 0},
					{"TANGENT", 0, DXGI_FORMAT_R32G32B32A32_FLOAT, 4, 0, D3D11_INPUT_PER_VERTEX_DATA, 0},
					};

				HRESULT hr = m_pDevice->CreateInputLayout(vertexFormat, 5, pVS->GetByteCode(), pVS->GetByteCodeSize(), &m_pMeshInputLayout);
				PLAY_ASSERT_MSG(SUCCEEDED(hr), "CreateInputLayout");
		}

		{
			Shader* pVS = Resources::ResourceManager<Shader>::Instance().GetPtr(m_skinnedMeshVS);

				D3D11_INPUT_ELEMENT_DESC vertexFormat[] = {
					{"POSITION", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 0, D3D11_INPUT_PER_VERTEX_DATA, 0},
					{"COLOUR", 0, DXGI_FORMAT_R8G8B8A8_UNORM, 1, 0, D3D11_INPUT_PER_VERTEX_DATA, 0},
					{"NORMAL", 0, DXGI_FORMAT_R32G32B32_FLOAT, 2, 0, D3D11_INPUT_PER_VERTEX_DATA, 0},
					{"UV", 0, DXGI_FORMAT_R32G32_FLOAT, 3, 0, D3D11_INPUT_PER_VERTEX_DATA, 0},
					{"TANGENT", 0, DXGI_FORMAT_R32G32B32A32_FLOAT, 4, 0, D3D11_INPUT_PER_VERTEX_DATA, 0},
					{"JOINTID", 0, DXGI_FORMAT_R8G8B8A8_UINT, 5, 0, D3D11_INPUT_PER_VERTEX_DATA, 0},
					{"JOINTWEIGHT", 0, DXGI_FORMAT_R32G32B32A32_FLOAT, 6, 0, D3D11_INPUT_PER_VERTEX_DATA, 0},
				};

				HRESULT hr = m_pDevice->CreateInputLayout(vertexFormat, 7, pVS->GetByteCode(), pVS->GetByteCodeSize(), &m_pSkinnedMeshInputLayout);
				PLAY_ASSERT_MSG(SUCCEEDED(hr), "CreateInputLayout");
		}

		return RESULT_OK;
	}

	result_t Graphics_Impl::CreatePipelineState()
	{
		HRESULT hr;
		{
			D3D11_RASTERIZER_DESC desc = {};
			desc.FillMode = D3D11_FILL_MODE::D3D11_FILL_SOLID;
			desc.CullMode = D3D11_CULL_MODE::D3D11_CULL_NONE;
			desc.FrontCounterClockwise = FALSE;
			desc.DepthBias = 0;
			desc.DepthBiasClamp = 0.f;
			desc.SlopeScaledDepthBias = 0.f;
			desc.DepthClipEnable = TRUE;
			desc.ScissorEnable = FALSE;
			desc.MultisampleEnable = FALSE;
			desc.AntialiasedLineEnable = FALSE;
			hr = m_pDevice->CreateRasterizerState(&desc, &m_pFallbackRasterState);
			PLAY_ASSERT_MSG(SUCCEEDED(hr), "Invalid RasterState");
		}
		{
			D3D11_DEPTH_STENCIL_DESC desc = {};
			desc.DepthEnable = TRUE;
			desc.DepthWriteMask = D3D11_DEPTH_WRITE_MASK::D3D11_DEPTH_WRITE_MASK_ALL;
			desc.DepthFunc = D3D11_COMPARISON_FUNC::D3D11_COMPARISON_LESS;
			desc.StencilEnable = FALSE;
			hr = m_pDevice->CreateDepthStencilState(&desc, &m_pFallbackDepthStencilState);
			PLAY_ASSERT_MSG(SUCCEEDED(hr), "Invalid DepthStencilState");
		}
		{
			D3D11_BLEND_DESC desc = {};
			desc.AlphaToCoverageEnable = FALSE;
			desc.IndependentBlendEnable = FALSE;
			desc.RenderTarget[0].BlendEnable = FALSE;
			desc.RenderTarget[0].SrcBlend = D3D11_BLEND_ONE;
			desc.RenderTarget[0].DestBlend = D3D11_BLEND_ZERO;
			desc.RenderTarget[0].BlendOp = D3D11_BLEND_OP_ADD;
			desc.RenderTarget[0].SrcBlendAlpha = D3D11_BLEND_ONE;
			desc.RenderTarget[0].DestBlendAlpha = D3D11_BLEND_ZERO;
			desc.RenderTarget[0].BlendOpAlpha = D3D11_BLEND_OP_ADD;
			desc.RenderTarget[0].RenderTargetWriteMask = D3D11_COLOR_WRITE_ENABLE_ALL;
			hr = m_pDevice->CreateBlendState(&desc, &m_pBlendStateOpaque);
			PLAY_ASSERT_MSG(SUCCEEDED(hr), "Invalid BlendState");

			desc.RenderTarget[0].BlendEnable = TRUE;
			desc.RenderTarget[0].SrcBlend = D3D11_BLEND_SRC_ALPHA;
			desc.RenderTarget[0].DestBlend = D3D11_BLEND_ONE;
			desc.RenderTarget[0].BlendOp = D3D11_BLEND_OP_ADD;
			desc.RenderTarget[0].SrcBlendAlpha = D3D11_BLEND_ONE;
			desc.RenderTarget[0].DestBlendAlpha = D3D11_BLEND_ZERO;
			desc.RenderTarget[0].BlendOpAlpha = D3D11_BLEND_OP_ADD;
			hr = m_pDevice->CreateBlendState(&desc, &m_pBlendStateAdditive);
			PLAY_ASSERT_MSG(SUCCEEDED(hr), "Invalid BlendState");

			desc.RenderTarget[0].BlendEnable = TRUE;
			desc.RenderTarget[0].SrcBlend = D3D11_BLEND_SRC_ALPHA;
			desc.RenderTarget[0].DestBlend = D3D11_BLEND_INV_SRC_ALPHA;
			desc.RenderTarget[0].BlendOp = D3D11_BLEND_OP_ADD;
			desc.RenderTarget[0].SrcBlendAlpha = D3D11_BLEND_ONE;
			desc.RenderTarget[0].DestBlendAlpha = D3D11_BLEND_ZERO;
			desc.RenderTarget[0].BlendOpAlpha = D3D11_BLEND_OP_ADD;
			hr = m_pDevice->CreateBlendState(&desc, &m_pBlendStateAlphaBlend);
			PLAY_ASSERT_MSG(SUCCEEDED(hr), "Invalid BlendState");
		}
		return RESULT_OK;
	}

	LRESULT CALLBACK Graphics_Impl::MainWndProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
	{
		Graphics_Impl* pImpl = reinterpret_cast<Graphics_Impl*>(GetWindowLongPtr(hWnd, GWLP_USERDATA));

		if (pImpl)
		{
			for (WindowCallback callback : pImpl->m_wndCallbacks)
			{
				callback(hWnd, message, wParam, lParam);
			}
		}

		switch (message)
		{
		case WM_CREATE: {
			LPCREATESTRUCT pCreateStruct = reinterpret_cast<LPCREATESTRUCT>(lParam);
			SetWindowLongPtr(hWnd, GWLP_USERDATA, reinterpret_cast<LONG_PTR>(pCreateStruct->lpCreateParams));
			return 0;
		}
		case WM_DESTROY: PostQuitMessage(0); break;
		case WM_SIZE: {
			u32 width = LOWORD(lParam);
			u32 height = HIWORD(lParam);

			if (width && height && pImpl)
			{
				pImpl->Resize(width, height);
			}
			return 0;
		}

		default: return DefWindowProc(hWnd, message, wParam, lParam);
		}
		return 0;
	}

	void Graphics_Impl::BindFrameConstants()
	{
		m_frameConstants.Bind(m_pDeviceContext.Get(), 0);
	}

	void Graphics_Impl::BindUIFrameConstants()
	{
		m_uiFrameConstants.Bind(m_pDeviceContext.Get(), 0);
	}

	void Graphics_Impl::BindActiveMaterial(ID3D11DeviceContext* pDC)
	{
		Material* pMaterial = Resources::ResourceManager<Material>::Instance().GetPtr(m_activeMaterial);
		InternalBindMaterial(pMaterial, pDC);
	}

	void Graphics_Impl::PushMarker(const char* name)
	{
		static constexpr u32 kMaxMarkerNameLength = 64;
		wchar_t wstrName[kMaxMarkerNameLength];
		size_t len = 0;
		mbstowcs_s(&len, wstrName, kMaxMarkerNameLength, name, _TRUNCATE);
		m_pAnnotation->BeginEvent(wstrName);
	}

	void Graphics_Impl::PopMarker()
	{
		m_pAnnotation->EndEvent();
	}

	void Graphics_Impl::ClearStateCache()
	{
		m_pPrevMaterial = nullptr;
		m_pPrevMesh = nullptr;
	}

	void Graphics_Impl::SetFullscreenMode(bool bEnable)
	{
		m_pSwapChain->SetFullscreenState(bEnable, NULL);
	}

}

//------------------------------------------- Graphics/Material.cpp -------------------------------------------

namespace Play3d::Graphics
{
	D3D11_CULL_MODE TranslateCullMode(CullMode mode)
	{
		switch (mode)
		{
		case CullMode::BACK: return D3D11_CULL_BACK;
		case CullMode::FRONT: return D3D11_CULL_FRONT;
		case CullMode::NONE: return D3D11_CULL_NONE;
		}
		return D3D11_CULL_NONE;
	}

	D3D11_COMPARISON_FUNC TranslateComparisonFunc(ComparisonFunc mode)
	{
		switch (mode)
		{
		case ComparisonFunc::NEVER: return D3D11_COMPARISON_NEVER;
		case ComparisonFunc::LESS: return D3D11_COMPARISON_LESS;
		case ComparisonFunc::LESSEQUAL: return D3D11_COMPARISON_LESS_EQUAL;
		case ComparisonFunc::EQUAL: return D3D11_COMPARISON_EQUAL;
		case ComparisonFunc::GREATEREQUAL: return D3D11_COMPARISON_GREATER_EQUAL;
		case ComparisonFunc::GREATER: return D3D11_COMPARISON_GREATER;
		case ComparisonFunc::ALWAYS: return D3D11_COMPARISON_ALWAYS;
		}
		return D3D11_COMPARISON_ALWAYS;
	}

	Material::Material(const SimpleMaterialDesc& rDesc)
		: m_bNullPixelShader(rDesc.m_bNullPixelShader)
	{
		auto pDevice = Graphics_Impl::Instance().GetDevice();
		SetupState(pDevice, rDesc.m_state);
		SetupConstantBuffer(pDevice, &rDesc.m_constants, sizeof(MaterialConstantData));
		SetupTextureBindings(pDevice, rDesc.m_texture, rDesc.m_sampler);

		MaterialShaderKey key;
		key.m_value = 0u;

		if (rDesc.m_bEnableLighting)
		{
			key.m_bits.m_useLighting = 1;
			PLAY_ASSERT_MSG(0 < rDesc.m_lightCount && rDesc.m_lightCount < MaterialShaderKey::kMaxLights,
							"Invalid light count.");
			key.m_bits.m_lightCount = rDesc.m_lightCount;
		}
		if (rDesc.m_texture[0].IsValid())
		{
			key.m_bits.m_useTexture0 = 1;
		}
		if (rDesc.m_texture[1].IsValid())
		{
			key.m_bits.m_useNormalMap = 1;
		}

		m_VertexShader = Graphics_Impl::Instance().GetMaterialShader(key);

		key.m_bits.m_pixelShader = 1;
		m_PixelShader = Graphics_Impl::Instance().GetMaterialShader(key);
		SetModified();
	}

	Material::Material(const ComplexMaterialDesc& rDesc)
		: m_bNullPixelShader(rDesc.m_bNullPixelShader)
	{
		auto pDevice = Graphics_Impl::Instance().GetDevice();
		SetupState(pDevice, rDesc.m_state);
		SetupConstantBuffer(pDevice, &rDesc.m_pConstantData, rDesc.m_dataSize);
		m_VertexShader = rDesc.m_VertexShader;
		m_PixelShader = rDesc.m_PixelShader;
		SetupTextureBindings(pDevice, rDesc.m_texture, rDesc.m_sampler);
		SetModified();
	}

	Material::~Material()
	{}

	void Material::SetTexture(u32 slot, TextureId id)
	{
		m_texture[slot] = id;
		SetModified();
	}

	void Material::SetupState(ID3D11Device* pDevice, const MaterialStateSettings& state)
	{
		{
			D3D11_RASTERIZER_DESC desc = {};
			desc.FillMode = state.m_fillMode == FillMode::SOLID ? D3D11_FILL_SOLID : D3D11_FILL_WIREFRAME;
			desc.CullMode = TranslateCullMode(state.m_cullMode);
			desc.FrontCounterClockwise = FALSE;
			desc.DepthBias = 0;
			desc.DepthBiasClamp = 0.f;
			desc.SlopeScaledDepthBias = 0.f;
			desc.DepthClipEnable = TRUE;
			desc.ScissorEnable = FALSE;
			desc.MultisampleEnable = FALSE;
			desc.AntialiasedLineEnable = FALSE;

			pDevice->CreateRasterizerState(&desc, &m_pRasterState);
		}

		{
			D3D11_DEPTH_STENCIL_DESC desc = {};
			desc.DepthEnable = state.m_depthEnable;
			desc.DepthWriteMask = state.m_depthWrite ? D3D11_DEPTH_WRITE_MASK_ALL : D3D11_DEPTH_WRITE_MASK_ZERO;
			desc.DepthFunc = TranslateComparisonFunc(state.m_depthComparison);
			pDevice->CreateDepthStencilState(&desc, &m_pDepthStencilState);
		}

		{
			D3D11_BLEND_DESC desc = {};
			desc.AlphaToCoverageEnable = false;
			desc.IndependentBlendEnable = false;
			desc.RenderTarget[0].BlendEnable = state.m_blendEnable;

			switch (state.m_blendMode)
			{
			case BlendMode::ADDITIVE:
				desc.RenderTarget[0].SrcBlend = D3D11_BLEND_SRC_ALPHA;
				desc.RenderTarget[0].DestBlend = D3D11_BLEND_ONE;
				break;
			case BlendMode::ALPHABLEND:
				desc.RenderTarget[0].SrcBlend = D3D11_BLEND_SRC_ALPHA;
				desc.RenderTarget[0].DestBlend = D3D11_BLEND_INV_SRC_ALPHA;
				break;
			case BlendMode::MULTIPLY:
				desc.RenderTarget[0].SrcBlend = D3D11_BLEND_ZERO;
				desc.RenderTarget[0].DestBlend = D3D11_BLEND_SRC_COLOR;
				break;
			default:
				desc.RenderTarget[0].SrcBlend = D3D11_BLEND_ONE;
				desc.RenderTarget[0].DestBlend = D3D11_BLEND_ZERO;
				break;
			};

			desc.RenderTarget[0].BlendOp = D3D11_BLEND_OP_ADD;
			desc.RenderTarget[0].SrcBlendAlpha = D3D11_BLEND_ONE;
			desc.RenderTarget[0].DestBlendAlpha = D3D11_BLEND_ZERO;
			desc.RenderTarget[0].BlendOpAlpha = D3D11_BLEND_OP_ADD;
			desc.RenderTarget[0].RenderTargetWriteMask = D3D11_COLOR_WRITE_ENABLE_ALL;

			pDevice->CreateBlendState(&desc, &m_pBlendState);
		}
	}

	void Material::SetupConstantBuffer(ID3D11Device* pDevice, const void* pData, size_t size)
	{
		if (pData && size > 0)
		{
			D3D11_BUFFER_DESC desc = {};
			desc.ByteWidth = (u32)size;
			desc.Usage = D3D11_USAGE::D3D11_USAGE_DYNAMIC;
			desc.BindFlags = D3D11_BIND_FLAG::D3D11_BIND_CONSTANT_BUFFER;
			desc.CPUAccessFlags = D3D11_CPU_ACCESS_WRITE;
			desc.MiscFlags = 0;
			desc.StructureByteStride = 0;

			D3D11_SUBRESOURCE_DATA data = {};
			data.pSysMem = pData;

			HRESULT hr = pDevice->CreateBuffer(&desc, &data, &m_pMaterialConstants);
			PLAY_ASSERT_MSG(SUCCEEDED(hr), "Could not create Material Constant Buffer");
		}
	}

	void Material::SetupTextureBindings(ID3D11Device* pDevice, const TextureId* pTextureId, const SamplerId* pSamplerId)
	{
		for (u32 i = 0; i < kMaxMaterialTextureSlots; ++i)
		{
			m_texture[i] = pTextureId[i];
			m_sampler[i] = pSamplerId[i];
		}
	}

	Play3d::Graphics::ComplexMaterialDesc& ComplexMaterialDesc::SetupFromHLSLFile(const char* name,
																				  const char* hlslPath)
	{
		size_t fileSizeBytes;
		const char* hlslCode = (const char*)System::LoadFileData(hlslPath, fileSizeBytes);
		PLAY_ASSERT_MSG(hlslCode, "Could not load the shader file");

		char shaderName[64];

		Graphics::ShaderId customVertexShader;
		{
			sprintf_s(shaderName, 64, "%s_VS", name);
			Graphics::ShaderCompilerDesc compilerOptions = {};
			compilerOptions.m_name = shaderName;
			compilerOptions.m_type = Graphics::ShaderType::VERTEX_SHADER;
			compilerOptions.m_flags = (u32)Graphics::ShaderCompilationFlags::DEBUG | (u32)Graphics::ShaderCompilationFlags::ADD_RUNTIME_COMPILE_MACRO;
			compilerOptions.m_hlslCode = hlslCode;
			compilerOptions.m_entryPoint = "VS_Main";
			compilerOptions.m_defines.push_back({"MAX_LIGHTS", "4"});
			customVertexShader = Graphics::Shader::Compile(compilerOptions);
			PLAY_ASSERT_MSG(customVertexShader.IsValid(), "Vertex Shader Compilation Failed!");
		}

		Graphics::ShaderId customPixelShader;
		{
			sprintf_s(shaderName, 64, "%s_PS", name);
			Graphics::ShaderCompilerDesc compilerOptions = {};
			compilerOptions.m_name = shaderName;
			compilerOptions.m_type = Graphics::ShaderType::PIXEL_SHADER;
			compilerOptions.m_flags = (u32)Graphics::ShaderCompilationFlags::DEBUG | (u32)Graphics::ShaderCompilationFlags::ADD_RUNTIME_COMPILE_MACRO;
			compilerOptions.m_hlslCode = hlslCode;
			compilerOptions.m_entryPoint = "PS_Main";
			compilerOptions.m_defines.push_back({"MAX_LIGHTS", "4"});
			customPixelShader = Graphics::Shader::Compile(compilerOptions);
			PLAY_ASSERT_MSG(customVertexShader.IsValid(), "Pixel Shader Compilation Failed!");
		}

		System::ReleaseFileData(const_cast<char*>(hlslCode));

		m_state.m_cullMode = Graphics::CullMode::BACK;
		m_state.m_fillMode = Graphics::FillMode::SOLID;
		m_VertexShader = customVertexShader;
		m_PixelShader = customPixelShader;

		return *this;
	}

	Play3d::Graphics::ComplexMaterialDesc& ComplexMaterialDesc::SetupFromCompiledShaders(const char* name, const char* fxoFolderPath)
	{
		std::string fxoVSPath = std::format("{}/{}_VS.fxo", fxoFolderPath,name);
		std::string fxoPSPath = std::format("{}/{}_PS.fxo", fxoFolderPath,name);

		m_state.m_cullMode = Graphics::CullMode::BACK;
		m_state.m_fillMode = Graphics::FillMode::SOLID;
		m_VertexShader = Graphics::Shader::LoadCompiledShaderFromFile(fxoVSPath.c_str(), ShaderType::VERTEX_SHADER);
		m_PixelShader = Graphics::Shader::LoadCompiledShaderFromFile(fxoPSPath.c_str(), ShaderType::PIXEL_SHADER);

		return *this;
	}

	Play3d::Graphics::ComplexMaterialDesc& ComplexMaterialDesc::SetupFromEitherHLSLorCompiledShaders(const char* name, const char* path)
	{
		std::string fxoVSPath = std::format("{}/{}_VS.fxo", path,name);
		std::string fxoPSPath = std::format("{}/{}_PS.fxo", path,name);

		if (System::CheckFileExists(fxoVSPath.c_str()) && System::CheckFileExists(fxoPSPath.c_str()))
		{
			this->SetupFromCompiledShaders(name, path);
		}
		else
		{
			std::string hlslPath = std::format("{}/{}.hlsl", path,name);
			this->SetupFromHLSLFile(name,hlslPath.c_str());
		}
		return *this;
	}

}

//--------------------------------------------- Graphics/Mesh.cpp ---------------------------------------------

namespace Play3d::Graphics
{
	D3D11_PRIMITIVE_TOPOLOGY TranslateMeshTopology(MeshTopology topology)
	{
		switch (topology)
		{
		case MeshTopology::POINT_LIST: return D3D11_PRIMITIVE_TOPOLOGY_POINTLIST;
		case MeshTopology::LINE_LIST: return D3D11_PRIMITIVE_TOPOLOGY_LINELIST;
		case MeshTopology::LINE_STRIP: return D3D11_PRIMITIVE_TOPOLOGY_LINESTRIP;
		case MeshTopology::TRI_LIST: return D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST;
		case MeshTopology::TRI_STRIP: return D3D11_PRIMITIVE_TOPOLOGY_TRIANGLESTRIP;
		}
		return D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST;
	}

	Mesh::Mesh(const MeshDesc& rDesc)
		: m_indexCount(rDesc.m_indexCount)
		, m_vertexCount(rDesc.m_vertexCount)
		, m_bIsSkinnedMesh(false)
		, m_topology(TranslateMeshTopology(rDesc.m_topology))
	{
		for (u32 i = 0; i < rDesc.m_streamCount; ++i)
		{
			const StreamInfo& rStream(rDesc.m_pStreams[i]);
			AddStream(rStream);
		}
	}

	Mesh::~Mesh()
	{
		for (auto& it : m_streamBuffers)
		{
			PLAY_SAFE_RELEASE(it);
		}
		PLAY_SAFE_RELEASE(m_pIndexBuffer);
	}

	void Mesh::AddStream(const StreamInfo& info)
	{
		bool bIsDynamic = (info.m_flags & StreamFlags::DYNAMIC_STREAM);

		D3D11_BUFFER_DESC desc = {};
		desc.ByteWidth = (UINT)info.m_dataSize;
		desc.Usage = bIsDynamic ? D3D11_USAGE::D3D11_USAGE_DYNAMIC : D3D11_USAGE_IMMUTABLE;
		desc.BindFlags = info.m_type == StreamType::INDEX ? D3D11_BIND_INDEX_BUFFER : D3D11_BIND_VERTEX_BUFFER;
		desc.CPUAccessFlags = bIsDynamic ? D3D11_CPU_ACCESS_WRITE : 0;
		desc.MiscFlags = 0;
		desc.StructureByteStride = 0;

		D3D11_SUBRESOURCE_DATA data = {};
		if (!bIsDynamic)
		{
			PLAY_ASSERT_MSG(info.m_pData, "A non-dynamic stream needs initial data.");
			data.pSysMem = info.m_pData;
		}

		ID3D11Buffer* pBuffer;
		HRESULT hr = Graphics_Impl::Instance().GetDevice()->CreateBuffer(&desc, bIsDynamic ? NULL : &data, &pBuffer);

		PLAY_ASSERT_MSG(SUCCEEDED(hr), "Create Vertex Buffer Stream");

		if (info.m_type == StreamType::INDEX)
		{
			m_pIndexBuffer = pBuffer;
		}
		else
		{
			m_streamInfos.push_back(info);
			m_streamBuffers.push_back(pBuffer);
			UINT stride;
			switch (info.m_type)
			{
			case StreamType::POSITION:
				stride = sizeof(f32) * 3;
				if (info.m_pData)
				{
					ComputeBoundsFromPositions(static_cast<const Vector3f*>(info.m_pData),
											   info.m_dataSize / stride,
											   m_bounds);
				}
				break;
			case StreamType::COLOUR:
				stride = sizeof(u32);
				break;
			case StreamType::NORMAL:
				stride = sizeof(f32) * 3;
				break;
			case StreamType::UV:
				stride = sizeof(f32) * 2;
				break;
			case StreamType::TANGENT:
				stride = sizeof(f32) * 4;
				break;
			case StreamType::JOINTID:
				stride = sizeof(u8) * 4;
				m_bIsSkinnedMesh = true;
				break;
			case StreamType::JOINTWEIGHT:
				stride = sizeof(f32) * 4;
				m_bIsSkinnedMesh = true;
				break;

			default: stride = info.m_stride; break;
			}
			m_strides.push_back(stride);
			m_offsets.push_back(info.m_offset);
		}
	}

	void Mesh::Bind(ID3D11DeviceContext* pDC) const
	{
		pDC->IASetPrimitiveTopology(m_topology);

		if (m_pIndexBuffer)
		{
			pDC->IASetIndexBuffer(m_pIndexBuffer, DXGI_FORMAT_R32_UINT, 0);
		}

		pDC->IASetVertexBuffers(0,
								(UINT)m_streamBuffers.size(),
								m_streamBuffers.data(),
								m_strides.data(),
								m_offsets.data());
	}
}

//------------------------------------------ Graphics/MeshBuilder.cpp ------------------------------------------

namespace Play3d::Graphics
{
	u32 MeshBuilder::AddVertex(const Vector3f& position, const Vector3f& normal, const Vector2f& uv, ColourValue colour)
	{
		u32 i0 = (u32)m_positions.size();
		m_positions.push_back(position);
		m_normals.push_back(normal);
		m_uvs.push_back(uv);
		m_colours.push_back(colour);
		return i0;
	}

	void MeshBuilder::AddFace(u32 vertexCount, const Vector3f* positions, ColourValue colour, bool flip /*= false*/)
	{
		u32 i0 = (u32)m_positions.size();

		Vector3f E0 = positions[1] - positions[0];
		Vector3f E1 = positions[2] - positions[0];
		Vector3f N = normalize(cross(E0, E1));

		if (flip)
			N = -N;

		Vector3f T = normalize(E0);
		Vector3f B = normalize(cross(T, N));

		for (u32 i = 0; i < vertexCount; ++i)
		{
			u32 j = flip ? i : vertexCount - i - 1;
			m_positions.push_back(positions[j]);
			m_colours.push_back(colour);
			m_normals.push_back(N);
			m_uvs.push_back(Vector2f(dot(T, positions[j]), dot(B, positions[j])));
		}

		for (u32 i = 1; i < (vertexCount - 1); ++i)
		{
			m_indices.push_back(i0);
			m_indices.push_back(i0 + i);
			m_indices.push_back(i0 + i + 1);
		}
	}

	void MeshBuilder::AddFace(u32 vertexCount, const Vector3f* positions, const ColourValue* colours,
							  const Vector3f* normals, const Vector2f* uvs, bool flip)
	{
		u32 i0 = (u32)m_positions.size();
		for (u32 i = 0; i < vertexCount; ++i)
		{
			u32 j = flip ? i : vertexCount - i - 1;
			const Vector3f& P(positions[j]);
			m_positions.push_back(positions[j]);
			m_colours.push_back(colours[j]);
			m_normals.push_back(normals[j]);
			m_uvs.push_back(uvs[j]);
		}

		for (u32 i = 1; i < (vertexCount - 1); ++i)
		{
			m_indices.push_back(i0);
			m_indices.push_back(i0 + i);
			m_indices.push_back(i0 + i + 1);
		}
	}

	void MeshBuilder::AddTriangle(u32 i0, u32 i1, u32 i2)
	{
		m_indices.push_back(i0);
		m_indices.push_back(i1);
		m_indices.push_back(i2);
	}

	void MeshBuilder::AddTriangle(const Vector3f& P0, const Vector3f& P1, const Vector3f& P2, ColourValue colour,
								  bool flip /*= false*/)
	{
		Vector3f positions[3] = {P0, P1, P2};
		AddFace(3, positions, colour, flip);
	}

	void MeshBuilder::AddQuad(u32 i0, u32 i1, u32 i2, u32 i3)
	{
		m_indices.push_back(i0);
		m_indices.push_back(i1);
		m_indices.push_back(i2);

		m_indices.push_back(i0);
		m_indices.push_back(i2);
		m_indices.push_back(i3);
	}

	void MeshBuilder::AddQuad(const Vector3f& P0, const Vector3f& P1, const Vector3f& P2, const Vector3f& P3,
							  ColourValue colour, bool flip /*= false*/)
	{
		Vector3f positions[4] = {P0, P1, P2, P3};
		AddFace(4, positions, colour, flip);
	}

	void MeshBuilder::AddFlatFace(u32 vertexCount, const Vector3f* positions, ColourValue colour, bool flip)
	{
		PLAY_ASSERT(vertexCount >= 3);

		const Vector3f& P0(positions[0]);
		const Vector3f& P1(positions[1]);
		const Vector3f& P2(positions[2]);

		Vector3f E0 = P1 - P0;
		Vector3f E1 = P2 - P0;
		Vector3f N = normalize(cross(E0, E1));

		if (flip)
			N = -N;

		Vector3f T = normalize(E0);
		Vector3f B = normalize(cross(T, N));

		u32 i0 = (u32)m_positions.size();
		for (u32 i = 0; i < vertexCount; ++i)
		{
			u32 j = flip ? i : vertexCount - i - 1;
			const Vector3f& P(positions[j]);
			m_positions.push_back(P);
			m_colours.push_back(colour);
			m_normals.push_back(N);
			m_uvs.push_back(Vector2f(dot(T, P), dot(B, P)));
		}

		for (u32 i = 1; i < (vertexCount - 1); ++i)
		{
			m_indices.push_back(i0);
			m_indices.push_back(i0 + i);
			m_indices.push_back(i0 + i + 1);
		}
	}

	void MeshBuilder::AddIndexedFlatFace(u32 indexCount, const u32* indices, const Vector3f* positions,
										 ColourValue colour, bool flip)
	{
		PLAY_ASSERT(indexCount >= 3);

		const Vector3f& P0(positions[indices[0]]);
		const Vector3f& P1(positions[indices[1]]);
		const Vector3f& P2(positions[indices[2]]);

		Vector3f E0 = P1 - P0;
		Vector3f E1 = P2 - P0;
		Vector3f N = normalize(cross(E0, E1));

		if (flip)
			N = -N;

		Vector3f T = normalize(E0);
		Vector3f B = normalize(cross(T, N));

		u32 i0 = (u32)m_positions.size();
		for (u32 i = 0; i < indexCount; ++i)
		{
			u32 j = flip ? i : indexCount - i - 1;
			const Vector3f& P(positions[indices[j]]);
			m_positions.push_back(P);
			m_colours.push_back(colour);
			m_normals.push_back(N);
			m_uvs.push_back(Vector2f(dot(T, P) + 0.5f, dot(B, P) + 0.5f));
		}

		for (u32 i = 1; i < (indexCount - 1); ++i)
		{
			m_indices.push_back(i0);
			m_indices.push_back(i0 + i);
			m_indices.push_back(i0 + i + 1);
		}
	}

	void MeshBuilder::Extrude(u32 vertexCount, const Vector3f* positions, ColourValue colour, f32 fLength)
	{
		std::vector<Vector3f> extrudedPositions(vertexCount);
		for (u32 i = 0; i < vertexCount; ++i)
		{
			extrudedPositions[i] = positions[i];
			extrudedPositions[i].y += fLength;
		}

		AddFlatFace(vertexCount, positions, colour, false);
		AddFlatFace(vertexCount, extrudedPositions.data(), colour, true);

		{
			u32 i = 0;
			for (; i < (vertexCount - 1); ++i)
			{
				AddQuad(positions[i], extrudedPositions[i], extrudedPositions[i + 1], positions[i + 1], colour, false);
			}
			AddQuad(positions[i], extrudedPositions[i], extrudedPositions[0], positions[0], colour, false);
		}
	}

	result_t MeshBuilder::ParseObjFormat(std::string_view objString, ColourValue colour, f32 fScale)
	{
		std::vector<Vector3f> positions;
		std::vector<Vector2f> texCoords;
		std::vector<Vector3f> normals;

		u32 nTotalFaceCount = 0;

		static constexpr u32 kMaxFaceVerts = 4;
		Vector3f facePositions[kMaxFaceVerts];
		Vector3f faceNormals[kMaxFaceVerts];
		Vector2f faceUVs[kMaxFaceVerts];
		ColourValue faceColours[kMaxFaceVerts];
		for (u32 i = 0; i < kMaxFaceVerts; ++i)
		{
			faceColours[i] = colour;
		}

		std::vector<std::string_view> args;

		size_t end = objString.size();
		size_t pos = 0;
		while (pos < end)
		{
			size_t offset = objString.find_first_of('\n', pos);
			if (std::string_view::npos == offset)
			{
				pos = end;
			}
			else if (offset > 0)
			{
				args.clear();
				size_t lineLength = offset - pos;
				std::string_view line = objString.substr(pos, lineLength);
				size_t argLen = line.find_first_of(' ', 0);
				while (argLen != std::string_view::npos)
				{
					args.push_back(line.substr(0, argLen));
					line.remove_prefix(argLen + 1);
					argLen = line.find_first_of(' ', 0);
				}
				if (!line.empty())
				{
					args.push_back(line);
				}

				if (!args.empty())
				{
					if ("v" == args[0] && args.size() > 3)
					{
						Vector3f v;
						std::from_chars(args[1].data(), args[1].data() + args[1].size(), v.x);
						std::from_chars(args[2].data(), args[2].data() + args[2].size(), v.y);
						std::from_chars(args[3].data(), args[3].data() + args[3].size(), v.z);
						positions.push_back(v);
					}
					else if ("vn" == args[0] && args.size() > 3)
					{
						Vector3f v;
						std::from_chars(args[1].data(), args[1].data() + args[1].size(), v.x);
						std::from_chars(args[2].data(), args[2].data() + args[2].size(), v.y);
						std::from_chars(args[3].data(), args[3].data() + args[3].size(), v.z);
						normals.push_back(v);
					}
					else if ("vt" == args[0] && args.size() > 2)
					{
						Vector2f v;
						std::from_chars(args[1].data(), args[1].data() + args[1].size(), v.x);
						std::from_chars(args[2].data(), args[2].data() + args[2].size(), v.y);
						v.y = 1.0f - v.y;
						texCoords.push_back(v);
					}
					else if ("f" == args[0] && args.size() > 3)
					{
						u32 nFaceVerts = (u32)args.size() - 1;
						PLAY_ASSERT(nFaceVerts <= kMaxFaceVerts);

						for (u32 i = 0; i < nFaceVerts; ++i)
						{
							const std::string_view& rArg(args[i + 1]);

							u32 nIndexArgs = 0;
							std::string_view indexArg[3];
							size_t t0 = rArg.find_first_of('/', 0);
							if (t0 != std::string_view::npos)
							{
								size_t t1 = rArg.find_first_of('/', t0 + 1);
								++nIndexArgs;
								if (t1 != std::string_view::npos)
								{
									++nIndexArgs;
									indexArg[0] = rArg.substr(0, t0);
									indexArg[1] = rArg.substr(t0 + 1, t1);
									indexArg[2] = rArg.substr(t1 + 1);
								}
								else
								{
									indexArg[0] = rArg.substr(0, t0);
									indexArg[1] = rArg.substr(t0 + 1);
								}
							}
							else
							{
								indexArg[0] = rArg;
							}

							u32 indices[3] = {0, 0, 0};
							for (u32 j = 0; j < 3; ++j)
							{
								if (!indexArg[0].empty())
								{
									std::from_chars(indexArg[j].data(),
													indexArg[j].data() + indexArg[j].size(),
													indices[j]);
								}
							}

							if (indices[0])
							{
								facePositions[i] = positions.at(indices[0] - 1) * fScale;
							}
							else
							{
								facePositions[i] = Vector3f(0, 0, 0);
							}

							if (indices[1])
							{
								faceUVs[i] = texCoords.at(indices[1] - 1);
							}
							else
							{
								faceUVs[i] = Vector2f(0, 0);
							}

							if (indices[2])
							{
								faceNormals[i] = normals.at(indices[2] - 1);
							}
							else
							{
								faceNormals[i] = Vector3f(0, 0, 0);
							}
						}

						AddFace(nFaceVerts, facePositions, faceColours, faceNormals, faceUVs);
						++nTotalFaceCount;
					}
					else
					{}
				}
				pos += lineLength + 1;
			}
			else
			{
				++pos;
			}
		}

		Debug::Printf("Parse OBJ v=%u vt=%u vn=%u f=%u\n",
					  (u32)positions.size(),
					  (u32)texCoords.size(),
					  (u32)normals.size(),
					  nTotalFaceCount);

		return RESULT_OK;
	}

	void MeshBuilder::GenerateTangents()
	{
		std::vector<Vector3f> tangents(m_positions.size(), Vector3f(0.0f, 0.0f, 0.0f));
		std::vector<Vector3f> bitangents(m_positions.size(), Vector3f(0.0f, 0.0f, 0.0f));

		size_t triangleCount = m_indices.size() / 3;

		for (size_t t = 0; t < triangleCount; ++t)
		{
			u32 TriIndex0 = m_indices[t * 3];
			u32 TriIndex1 = m_indices[t * 3 + 1];
			u32 TriIndex2 = m_indices[t * 3 + 2];

			const Vector3f& Point0 = m_positions[TriIndex0];
			const Vector3f& Point1 = m_positions[TriIndex1];
			const Vector3f& Point2 = m_positions[TriIndex2];

			const Vector2f& Point0UVs = m_uvs[TriIndex0];
			const Vector2f& Point1UVs = m_uvs[TriIndex1];
			const Vector2f& Point2UVs = m_uvs[TriIndex2];

			Vector3f e1 = Point1 - Point0;
			Vector3f e2 = Point2 - Point0;

			float x1 = Point1UVs.x - Point0UVs.x;
			float x2 = Point2UVs.x - Point0UVs.x;
			float y1 = Point1UVs.y - Point0UVs.y;
			float y2 = Point2UVs.y - Point0UVs.y;

			float r = 1.0f / (x1 * y2 - x2 * y1);
			Vector3f tangent = (e1 * y2 - e2 * y1) * r;
			Vector3f bitangent = (e2 * x1 - e1 * x2) * r;

			tangents[TriIndex0] += tangent;
			tangents[TriIndex1] += tangent;
			tangents[TriIndex2] += tangent;

			bitangents[TriIndex0] += bitangent;
			bitangents[TriIndex1] += bitangent;
			bitangents[TriIndex2] += bitangent;
		}

		for (u32 v = 0; v < m_positions.size(); ++v)
		{
			const Vector3f& tangent = tangents[v];
			const Vector3f& bitangent = bitangents[v];
			const Vector3f& normal = m_normals[v];

			Vector3f newTangent;

			newTangent = normalize(tangent - dot(tangent, normal) * normal);
			Vector4f test;
			float handedness = (dot(cross(tangent, bitangent), normal) > 0.0f) ? 1.0f : -1.0f;

			m_tangents.push_back(Vector4f(newTangent, handedness));
		}
	}

	MeshId MeshBuilder::CreateMesh()
	{
		MeshDesc desc;
		StreamInfo streamInfos[6];
		streamInfos[0].m_type = StreamType::POSITION;
		streamInfos[0].m_pData = m_positions.data();
		streamInfos[0].m_dataSize = m_positions.size() * sizeof(Vector3f);
		streamInfos[0].m_stride = sizeof(Vector3f);

		streamInfos[1].m_type = StreamType::COLOUR;
		streamInfos[1].m_pData = m_colours.data();
		streamInfos[1].m_dataSize = m_colours.size() * sizeof(u32);
		streamInfos[1].m_stride = sizeof(u32);

		streamInfos[2].m_type = StreamType::NORMAL;
		streamInfos[2].m_pData = m_normals.data();
		streamInfos[2].m_dataSize = m_normals.size() * sizeof(Vector3f);
		streamInfos[2].m_stride = sizeof(Vector3f);

		streamInfos[3].m_type = StreamType::UV;
		streamInfos[3].m_pData = m_uvs.data();
		streamInfos[3].m_dataSize = m_uvs.size() * sizeof(Vector2f);
		streamInfos[3].m_stride = sizeof(Vector2f);

		streamInfos[4].m_type = StreamType::TANGENT;
		streamInfos[4].m_pData = m_tangents.data();
		streamInfos[4].m_dataSize = m_tangents.size() * sizeof(Vector4f);
		streamInfos[4].m_stride = sizeof(Vector4f);

		streamInfos[5].m_type = StreamType::INDEX;
		streamInfos[5].m_pData = m_indices.data();
		streamInfos[5].m_dataSize = m_indices.size() * sizeof(u32);
		streamInfos[5].m_stride = sizeof(u32);

		desc.m_pStreams = streamInfos;
		desc.m_streamCount = 6;
		desc.m_vertexCount = (u32)m_positions.size();
		desc.m_indexCount = (u32)m_indices.size();

		return Resources::CreateAsset<Mesh>(desc);
	}

	void MeshBuilder::Reset()
	{
		m_positions.clear();
		m_colours.clear();
		m_normals.clear();
		m_uvs.clear();
		m_indices.clear();
		m_tangents.clear();
	}
}

//-------------------------------------- Graphics/PrimitiveBatch_Impl.cpp --------------------------------------

namespace Play3d::Graphics
{
	PLAY_SINGLETON_IMPL(PrimitiveRenderSystem);

	PrimitiveBatch::PrimitiveBatch(ID3D11Device* pDevice, u32 kMaxVertexCount)
		: m_totalVertexCount(0)
		, m_maxVertexCount(kMaxVertexCount)
		, m_pointVertexCount(0)
		, m_lineVertexCount(0)
		, m_triangleVertexCount(0)
	{
		D3D11_BUFFER_DESC desc = {};
		desc.ByteWidth = sizeof(PrimitiveVertex) * kMaxVertexCount;
		desc.Usage = D3D11_USAGE::D3D11_USAGE_DYNAMIC;
		desc.BindFlags = D3D11_BIND_FLAG::D3D11_BIND_VERTEX_BUFFER;
		desc.CPUAccessFlags = D3D11_CPU_ACCESS_WRITE;
		desc.MiscFlags = 0;
		desc.StructureByteStride = 0;
		HRESULT hr = pDevice->CreateBuffer(&desc, NULL, &m_pVertexBuffer);
		PLAY_ASSERT_MSG(SUCCEEDED(hr), "CreateBuffer");
	}

	PrimitiveBatch::~PrimitiveBatch()
	{}

	void PrimitiveBatch::AppendPoint(const Vector3f& v1, ColourValue c1)
	{
		if ((m_totalVertexCount + 1) < m_maxVertexCount)
		{
			m_points.push_back({v1, c1});
			m_totalVertexCount += 1;
		}
	}

	void PrimitiveBatch::AppendLine(const Vector3f& v1, const Vector3f& v2, ColourValue c1, ColourValue c2)
	{
		m_lines.push_back({v1, c1});
		m_lines.push_back({v2, c2});
	}

	void PrimitiveBatch::AppendTriangle(const Vector3f& v1, const Vector3f& v2, const Vector3f& v3, ColourValue c1,
										ColourValue c2, ColourValue c3)
	{
		m_triangles.push_back({v1, c1});
		m_triangles.push_back({v2, c2});
		m_triangles.push_back({v3, c3});
	}

	void PrimitiveBatch::Flush(ID3D11DeviceContext* pContext)
	{
		D3D11_MAPPED_SUBRESOURCE data;
		HRESULT hr = pContext->Map(m_pVertexBuffer.Get(), 0, D3D11_MAP::D3D11_MAP_WRITE_DISCARD, 0, &data);
		if (SUCCEEDED(hr))
		{
			m_pointVertexCount = (u32)m_points.size();
			m_lineVertexCount = (u32)m_lines.size();
			m_triangleVertexCount = (u32)m_triangles.size();

			PrimitiveVertex* pVertexOut = (PrimitiveVertex*)data.pData;
			memcpy(pVertexOut, m_points.data(), m_pointVertexCount * sizeof(PrimitiveVertex));
			pVertexOut += m_pointVertexCount;

			memcpy(pVertexOut, m_lines.data(), m_lineVertexCount * sizeof(PrimitiveVertex));
			pVertexOut += m_lineVertexCount;

			memcpy(pVertexOut, m_triangles.data(), m_triangleVertexCount * sizeof(PrimitiveVertex));

			pContext->Unmap(m_pVertexBuffer.Get(), 0);

			m_totalVertexCount = 0;
			m_points.clear();
			m_lines.clear();
			m_triangles.clear();
		}
	}

	void PrimitiveBatch::Bind(ID3D11DeviceContext* pContext)
	{
		ID3D11Buffer* buffers[] = {m_pVertexBuffer.Get()};
		UINT strides[] = {sizeof(PrimitiveVertex)};
		UINT offsets[] = {0};

		pContext->IASetVertexBuffers(0, 1, buffers, strides, offsets);
	}

	void PrimitiveBatch::DrawPoints(ID3D11DeviceContext* pContext)
	{
		if (m_pointVertexCount > 0)
		{
			pContext->IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_POINTLIST);
			pContext->Draw(m_pointVertexCount, 0);
		}
	}

	void PrimitiveBatch::DrawLines(ID3D11DeviceContext* pContext)
	{
		if (m_lineVertexCount > 0)
		{
			pContext->IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_LINELIST);
			pContext->Draw(m_lineVertexCount, m_pointVertexCount);
		}
	}

	void PrimitiveBatch::DrawTriangles(ID3D11DeviceContext* pContext)
	{
		if (m_triangleVertexCount > 0)
		{
			pContext->IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
			pContext->Draw(m_triangleVertexCount, m_pointVertexCount + m_lineVertexCount);
		}
	}

	PrimitiveRenderSystem::PrimitiveRenderSystem()
		: m_nNextPrimitiveBatch(0)
	{
		HRESULT hr;

		{
			u32 compilationFlags = 0;
#ifdef _DEBUG
			compilationFlags |= ShaderCompilationFlags::DEBUG;
#endif

			ShaderCompilerDesc desc;
			desc.m_name = "HLSL_PrimitiveBatchShader";
			desc.m_hlslCode = HLSL_PrimitiveBatchShader;
			desc.m_type = ShaderType::VERTEX_SHADER;
			desc.m_entryPoint = "VS_Main";
			desc.m_flags = compilationFlags;

			m_primitiveBatchVS = Shader::Compile(desc);

			desc.m_type = ShaderType::PIXEL_SHADER;
			desc.m_entryPoint = "PS_Main";

			m_primitiveBatchPS = Shader::Compile(desc);
		}

		auto pDevice = Graphics_Impl::Instance().GetDevice();

		{
			Shader* pVS = Resources::ResourceManager<Shader>::Instance().GetPtr(m_primitiveBatchVS);

			D3D11_INPUT_ELEMENT_DESC vertexFormat[] = {
				{"POSITION",0,DXGI_FORMAT_R32G32B32_FLOAT,0,offsetof(PrimitiveVertex, position), D3D11_INPUT_PER_VERTEX_DATA, 0},
				{"COLOUR",0,DXGI_FORMAT_R8G8B8A8_UNORM, 0,offsetof(PrimitiveVertex, colour),D3D11_INPUT_PER_VERTEX_DATA,0},
			};

			hr = pDevice->CreateInputLayout(vertexFormat,2, pVS->GetByteCode(),pVS->GetByteCodeSize(),&m_pPrimitiveInputLayout);
			PLAY_ASSERT_MSG(SUCCEEDED(hr), "CreateInputLayout");
		}

		{
			D3D11_RASTERIZER_DESC desc = {};
			desc.FillMode = D3D11_FILL_MODE::D3D11_FILL_SOLID;
			desc.CullMode = D3D11_CULL_MODE::D3D11_CULL_NONE;
			desc.FrontCounterClockwise = FALSE;
			desc.DepthBias = 0;
			desc.DepthBiasClamp = 0.f;
			desc.SlopeScaledDepthBias = 0.f;
			desc.DepthClipEnable = TRUE;
			desc.ScissorEnable = FALSE;
			desc.MultisampleEnable = FALSE;
			desc.AntialiasedLineEnable = FALSE;
			hr = pDevice->CreateRasterizerState(&desc, &m_pRasterState);
			PLAY_ASSERT_MSG(SUCCEEDED(hr), "Invalid RasterState");
		}
		{
			D3D11_DEPTH_STENCIL_DESC desc = {};
			desc.DepthEnable = TRUE;
			desc.DepthWriteMask = D3D11_DEPTH_WRITE_MASK::D3D11_DEPTH_WRITE_MASK_ALL;
			desc.DepthFunc = D3D11_COMPARISON_FUNC::D3D11_COMPARISON_LESS;
			desc.StencilEnable = FALSE;
			hr = pDevice->CreateDepthStencilState(&desc, &m_pDepthStencilState);
			PLAY_ASSERT_MSG(SUCCEEDED(hr), "Invalid DepthStencilState");
		}
		{
			D3D11_BLEND_DESC desc = {};
			desc.AlphaToCoverageEnable = FALSE;
			desc.IndependentBlendEnable = FALSE;
			desc.RenderTarget[0].BlendEnable = FALSE;
			desc.RenderTarget[0].SrcBlend = D3D11_BLEND_ONE;
			desc.RenderTarget[0].DestBlend = D3D11_BLEND_ZERO;
			desc.RenderTarget[0].BlendOp = D3D11_BLEND_OP_ADD;
			desc.RenderTarget[0].SrcBlendAlpha = D3D11_BLEND_ONE;
			desc.RenderTarget[0].DestBlendAlpha = D3D11_BLEND_ZERO;
			desc.RenderTarget[0].BlendOpAlpha = D3D11_BLEND_OP_ADD;
			desc.RenderTarget[0].RenderTargetWriteMask = D3D11_COLOR_WRITE_ENABLE_ALL;
			hr = pDevice->CreateBlendState(&desc, &m_pBlendState);
			PLAY_ASSERT_MSG(SUCCEEDED(hr), "Invalid BlendState");
		}
	}

	PrimitiveRenderSystem::~PrimitiveRenderSystem()
	{
		for (auto& it : m_primitiveBatchRing)
		{
			PLAY_SAFE_DELETE(it);
		}
	}

	void PrimitiveRenderSystem::OnBeginFrame()
	{
		m_nNextPrimitiveBatch = 0;
	}

	void PrimitiveRenderSystem::OnEndFrame()
	{}

	PrimitiveBatch* PrimitiveRenderSystem::AllocatePrimitiveBatch()
	{
		PrimitiveBatch* pBatch = nullptr;
		if (m_nNextPrimitiveBatch < m_primitiveBatchRing.size())
		{
			pBatch = m_primitiveBatchRing[m_nNextPrimitiveBatch];
			++m_nNextPrimitiveBatch;
		}
		else
		{
			auto pDevice = Graphics_Impl::Instance().GetDevice();
			pBatch = new PrimitiveBatch(pDevice, 0x10000);
			m_primitiveBatchRing.push_back(pBatch);
			++m_nNextPrimitiveBatch;
		}
		PLAY_ASSERT(pBatch);
		return pBatch;
	}

	void PrimitiveRenderSystem::DrawPrimitiveBatch(PrimitiveBatch* pBatch)
	{
		PLAY_ASSERT(pBatch);

		Graphics_Impl::Instance().UpdateConstantBuffers();

		ID3D11DeviceContext* pDC = Graphics_Impl::Instance().GetDeviceContext();

		pBatch->Flush(pDC);

		Shader* pVS = Resources::ResourceManager<Shader>::Instance().GetPtr(m_primitiveBatchVS);
		Shader* pPS = Resources::ResourceManager<Shader>::Instance().GetPtr(m_primitiveBatchPS);

		pDC->IASetInputLayout(m_pPrimitiveInputLayout.Get());

		pVS->Bind(pDC);
		pPS->Bind(pDC);

		pBatch->Bind(pDC);

		Graphics_Impl::Instance().BindFrameConstants();

		pDC->RSSetState(m_pRasterState.Get());
		pDC->OMSetBlendState(m_pBlendState.Get(), NULL, 0xffffffff);
		pDC->OMSetDepthStencilState(m_pDepthStencilState.Get(), 0);

		pBatch->DrawPoints(pDC);
		pBatch->DrawLines(pDC);
		pBatch->DrawTriangles(pDC);
	}

}

//-------------------------------------------- Graphics/Sampler.cpp --------------------------------------------

namespace Play3d::Graphics
{
	D3D11_FILTER TranslateFilterMode(FilterMode mode)
	{
		switch (mode)
		{
		case FilterMode::BILINEAR: return D3D11_FILTER_MIN_MAG_MIP_LINEAR;
		case FilterMode::POINT: return D3D11_FILTER_MIN_MAG_MIP_POINT;
		case FilterMode::ANISOTROPIC: return D3D11_FILTER_ANISOTROPIC;
		default: return D3D11_FILTER_MIN_MAG_MIP_POINT;
		};
	}

	D3D11_TEXTURE_ADDRESS_MODE TranslateAddressMode(AddressMode mode)
	{
		switch (mode)
		{
		case AddressMode::CLAMP: return D3D11_TEXTURE_ADDRESS_CLAMP;
		case AddressMode::WRAP: return D3D11_TEXTURE_ADDRESS_WRAP;
		case AddressMode::MIRROR: return D3D11_TEXTURE_ADDRESS_MIRROR;
		case AddressMode::MIRROR_ONCE: return D3D11_TEXTURE_ADDRESS_MIRROR_ONCE;
		case AddressMode::BORDER: return D3D11_TEXTURE_ADDRESS_BORDER;
		default: return D3D11_TEXTURE_ADDRESS_CLAMP;
		};
	}

	Sampler::Sampler(const SamplerDesc& rDesc)
	{
		ID3D11Device* pDevice = Graphics::Graphics_Impl::Instance().GetDevice();

		D3D11_SAMPLER_DESC desc = {};
		desc.Filter = TranslateFilterMode(rDesc.m_filter);
		desc.AddressU = TranslateAddressMode(rDesc.m_addressModeU);
		desc.AddressV = TranslateAddressMode(rDesc.m_addressModeV);
		desc.AddressW = TranslateAddressMode(rDesc.m_addressModeW);
		desc.MinLOD = -FLT_MAX;
		desc.MaxLOD = FLT_MAX;
		desc.MipLODBias = 0.0;
		desc.MaxAnisotropy = rDesc.m_filter == FilterMode::ANISOTROPIC ? 16 : 1;
		desc.ComparisonFunc = D3D11_COMPARISON_NEVER;

		rDesc.m_borderColour.as_float_rgba_srgb(desc.BorderColor);

		HRESULT hr = pDevice->CreateSamplerState(&desc, &m_pSampler);
		PLAY_ASSERT(SUCCEEDED(hr));
	}
}

//-------------------------------------------- Graphics/Shader.cpp --------------------------------------------

#pragma comment(lib, "d3dcompiler.lib")

namespace Play3d::Graphics
{
	Shader::Shader(const ShaderDesc& rDesc)
		: m_shaderType(rDesc.m_type)
		, m_pByteCode(nullptr)
		, m_sizeBytes(0)
	{
		auto pDevice(Graphics_Impl::Instance().GetDevice());

		HRESULT hr;

		std::string name(rDesc.m_name);

		switch (rDesc.m_type)
		{
		case ShaderType::VERTEX_SHADER: {
			ComPtr<ID3D11VertexShader> pVertexShader;
			hr = pDevice->CreateVertexShader(rDesc.m_pByteCode, rDesc.m_sizeBytes, NULL, &pVertexShader);
			PLAY_ASSERT_MSG(SUCCEEDED(hr), "Failed to create vertex shader.");

			hr = pVertexShader.As(&m_pShader);
			PLAY_ASSERT(SUCCEEDED(hr));

			name += "_VS";
			break;
		}
		case ShaderType::PIXEL_SHADER: {
			ComPtr<ID3D11PixelShader> pPixelShader;
			hr = pDevice->CreatePixelShader(rDesc.m_pByteCode, rDesc.m_sizeBytes, NULL, &pPixelShader);
			PLAY_ASSERT_MSG(SUCCEEDED(hr), "Failed to create pixel shader.");

			hr = pPixelShader.As(&m_pShader);
			PLAY_ASSERT(SUCCEEDED(hr));

			name += "_PS";
			break;
		}
		case ShaderType::COMPUTE_SHADER: {
			ComPtr<ID3D11ComputeShader> pComputeShader;
			hr = pDevice->CreateComputeShader(rDesc.m_pByteCode, rDesc.m_sizeBytes, NULL, &pComputeShader);
			PLAY_ASSERT_MSG(SUCCEEDED(hr), "Failed to create compute shader.");

			hr = pComputeShader.As(&m_pShader);
			PLAY_ASSERT(SUCCEEDED(hr));

			name += "_CS";
			break;
		}

		default: PLAY_ASSERT(false);
		};

		m_pByteCode = _aligned_malloc(rDesc.m_sizeBytes, 16);
		PLAY_ASSERT(m_pByteCode);
		memcpy(m_pByteCode, rDesc.m_pByteCode, rDesc.m_sizeBytes);
		m_sizeBytes = rDesc.m_sizeBytes;

		hr = m_pShader->SetPrivateData(WKPDID_D3DDebugObjectName, (UINT)name.size(), name.data());
		PLAY_ASSERT(SUCCEEDED(hr));
	}

	Shader::~Shader()
	{
		if (m_pByteCode)
		{
			_aligned_free(m_pByteCode);
			m_pByteCode = nullptr;
		}
	}

	void Shader::Bind(ID3D11DeviceContext* pContext)
	{
		HRESULT hr;
		switch (m_shaderType)
		{
		case ShaderType::VERTEX_SHADER: {
			ComPtr<ID3D11VertexShader> pVertexShader;
			hr = m_pShader.As(&pVertexShader);
			PLAY_ASSERT(SUCCEEDED(hr));
			pContext->VSSetShader(pVertexShader.Get(), nullptr, 0);
			break;
		}
		case ShaderType::PIXEL_SHADER: {
			ComPtr<ID3D11PixelShader> pPixelShader;
			hr = m_pShader.As(&pPixelShader);
			PLAY_ASSERT(SUCCEEDED(hr));
			pContext->PSSetShader(pPixelShader.Get(), nullptr, 0);
			break;
		}
		case ShaderType::COMPUTE_SHADER: {
			ComPtr<ID3D11ComputeShader> pComputeShader;
			hr = m_pShader.As(&pComputeShader);
			PLAY_ASSERT(SUCCEEDED(hr));
			pContext->CSSetShader(pComputeShader.Get(), nullptr, 0);
			break;
		}
		default: PLAY_ASSERT(false);
		};
	}

	ShaderId Shader::Compile(const ShaderCompilerDesc& rDesc)
	{
		const bool bCompileFromFile = ((rDesc.m_flags & ShaderCompilationFlags::SOURCE_FILE) == ShaderCompilationFlags::SOURCE_FILE);
		const bool bAddRuntimeCompileMacro = ((rDesc.m_flags & ShaderCompilationFlags::ADD_RUNTIME_COMPILE_MACRO) == ShaderCompilationFlags::ADD_RUNTIME_COMPILE_MACRO);
		const bool bUseStandardMacros = ((rDesc.m_flags & ShaderCompilationFlags::DISABLE_STANDARD_MACROS) == 0);

		if (bCompileFromFile)
		{
			Debug::Printf("Compiling %s : %s ('%s')\n", rDesc.m_name.c_str(), rDesc.m_entryPoint.c_str(), rDesc.m_hlslCode.c_str());
		}
		else
		{
			Debug::Printf("Compiling %s : %s (Embedded Code)\n", rDesc.m_name.c_str(), rDesc.m_entryPoint.c_str());
		}

		UINT flags = D3DCOMPILE_ENABLE_STRICTNESS;
		if (rDesc.m_flags & ShaderCompilationFlags::DEBUG)
		{
			flags |= D3DCOMPILE_DEBUG;
		}

		const char* pTarget;
		switch (rDesc.m_type)
		{
		case ShaderType::VERTEX_SHADER: pTarget = "vs_5_0"; break;
		case ShaderType::PIXEL_SHADER: pTarget = "ps_5_0"; break;
		case ShaderType::COMPUTE_SHADER: pTarget = "cs_5_0"; break;
		default: pTarget = "vs_5_0";
		};

		std::vector<D3D_SHADER_MACRO> defines = rDesc.m_defines;
		if(bAddRuntimeCompileMacro)
		{
			defines.push_back({"PLAY3D_RUNTIME_COMPILE", NULL});
		}
		defines.push_back({NULL, NULL});

		ComPtr<ID3DBlob> pShaderBlob;
		ComPtr<ID3DBlob> pErrorBlob;

		std::string strSourceCode;
		if (bUseStandardMacros)
		{
			strSourceCode += std::string("#line 1 \"HLSL_StandardMacros\"\n");
			strSourceCode += Graphics::HLSL_StandardMacros;
		}

		const char* pLoadedSrc = nullptr;
		if (bCompileFromFile)
		{
			size_t sourceSizeBytes = 0;
			pLoadedSrc = (const char*)System::LoadFileData(rDesc.m_hlslCode.c_str(), sourceSizeBytes);
			PLAY_ASSERT_MSG(pLoadedSrc, "Could not load the shader file '%s'", rDesc.m_hlslCode.c_str());

			strSourceCode += std::string("#line 1 \"") + rDesc.m_hlslCode + std::string("\"\n");
			strSourceCode += std::string(pLoadedSrc);
		}
		else
		{
			strSourceCode += std::string("#line 1 \"HLSL_Internal\"\n");
			strSourceCode += rDesc.m_hlslCode;
		}

		HRESULT hr = D3DCompile(strSourceCode.data(),
								strSourceCode.size(),
								bCompileFromFile ? rDesc.m_hlslCode.c_str() : NULL,
								defines.data(),
								bCompileFromFile ? D3D_COMPILE_STANDARD_FILE_INCLUDE : NULL,
								rDesc.m_entryPoint.c_str(),
								pTarget,
								flags,
								0,
								&pShaderBlob,
								&pErrorBlob);

		if (bCompileFromFile)
		{
			System::ReleaseFileData(const_cast<char*>(pLoadedSrc));
		}

		if (FAILED(hr))
		{
			if (pErrorBlob)
			{
				Debug::Printf("!! Shader Compilation Failed !! : "
							  "\n--------------------------------\n\n%s\n\n--------------------------------",
							  (const char*)pErrorBlob->GetBufferPointer());
			}
			return ShaderId();
		}

		ShaderDesc desc;
		desc.m_pByteCode = pShaderBlob->GetBufferPointer();
		desc.m_sizeBytes = pShaderBlob->GetBufferSize();
		desc.m_type = rDesc.m_type;
		desc.m_name = rDesc.m_name;

		return Resources::CreateAsset<Shader>(desc);
	}

	ShaderId Shader::LoadCompiledShaderFromFile(const char* pFilePath, ShaderType type)
	{

		size_t sourceSizeBytes = 0;
		const void* pLoadedSrc = (const char*)System::LoadFileData(pFilePath, sourceSizeBytes);

		if (!pLoadedSrc)
		{
			Debug::Printf("!! Shader Load Failed !!");
			return ShaderId();
		}

		ShaderDesc desc;
		desc.m_name = pFilePath;
		desc.m_pByteCode = const_cast<void*>(pLoadedSrc);
		desc.m_sizeBytes = sourceSizeBytes;
		desc.m_type = type;

		ShaderId asset =  Resources::CreateAsset<Shader>(desc);

		System::ReleaseFileData(pLoadedSrc);
		return asset;

	}

}

//---------------------------------------- Graphics/ShaderCode_Impl.cpp ----------------------------------------

namespace Play3d::Graphics
{

//!-------------------------------------- EmbeddedShaders/FontShader.hlsl --------------------------------------
const char* HLSL_FontShader = R"(

cbuffer UIFrameConstantData : register(b0)
{
	float4x4 viewProjectionMtx;
	float4 viewportRect;
};

struct VSInput
{
	float2 position : POSITION;
	float2 uv : UV;
	float4 colour : COLOUR;
};

struct PSInput
{
	float4 position : SV_POSITION;
	float4 colour : COLOUR;
	float2 uv : UV;
};

PSInput VS_Main(VSInput input)
{
	PSInput output;

	output.position = mul(viewProjectionMtx, float4(input.position.xy, -0.1f, 1.0f));
	output.uv = input.uv;
	output.colour = pow(input.colour, 2.2);

	return output;
}

Texture2D g_texture0 : register(t0);
SamplerState g_sampler0 : register(s0);

float4 PS_Main(PSInput input) : SV_TARGET
{
	float alpha = g_texture0.Sample(g_sampler0, input.uv).r;
	return input.colour * float4(1,1,1,alpha);
}

)";
//!-------------------------------------- EmbeddedShaders/MeshShader.hlsl --------------------------------------
const char* HLSL_MeshShader = R"(
PLAY3D_PER_FRAME_CONSTANTS

PLAY3D_PER_DRAW_CONSTANTS

PLAY3D_LIGHT_CONSTANTS

PLAY3D_MATERIAL_CONSTANTS

PLAY3D_MESH_VS_INPUTS

struct PSInput
{
	float4 position : SV_POSITION;
	float4 colour : COLOUR;
	float3 normal : NORMAL;
	float2 uv : UV;
	float4 tangent: TANGENT;
};

PSInput VS_Main(VSInput input)
{
	PSInput output;

	output.position = mul(mvpMtx, float4(input.position.xyz, 1.0f));
	output.colour = pow(input.colour, 2.2);
	output.normal = mul(normalMtx, float4(input.normal.xyz, 0.0f));
	output.uv = input.uv;
	output.tangent.xyz = mul(normalMtx, float4(input.tangent.xyz, 0.0f));
	output.tangent.w = input.tangent.w;

	return output;
}

#if USE_TEXTURE_0
Texture2D g_texture0 : register(t0);
SamplerState g_sampler0 : register(s0);
#endif

#if USE_NORMAL_TEXTURE
Texture2D g_texture1 : register(t1);
SamplerState g_sampler1 : register(s1);
#endif

float3 GetNormalDirection(PSInput input)
{
#if USE_NORMAL_TEXTURE
	float3 normalTexture = g_texture1.Sample(g_sampler1, input.uv);
	normalTexture = saturate(pow(normalTexture, 0.4545f));
	normalTexture = (normalTexture * 2.0f) - 1.0f;

	float3 N = normalize(input.normal);
	float3 T = normalize(input.tangent.xyz);
	float3 B = normalize(cross(N, T) * input.tangent.w);
	float3x3 TBN = float3x3(T, B, N);

	N = mul(normalTexture, TBN);
#else
	float3 N = normalize(input.normal);
#endif
	return N;
}

float4 PS_Main(PSInput input) : SV_TARGET
{
#if USE_TEXTURE_0
	float4 vTexture0 = g_texture0.Sample(g_sampler0, input.uv);
	float4 vAlbedo = input.colour * diffuseColour * vTexture0;
#else
	float4 vAlbedo = input.colour * diffuseColour;
#endif

#if USE_LIGHTING

	float3 N = GetNormalDirection(input);

	float3 vDiffuse = float3(0, 0, 0);

	[unroll] for (uint i = 0; i < LIGHT_COUNT; ++i)
	{
		float3 L = lightDir[i];
		float I = max(dot(N, L), 0);
		vDiffuse += lightColour[i] * I;
	}
	float3 vAmbient = lightAmbient.xyz;
#else
	float3 vDiffuse = float3(1.0, 1.0, 1.0);
	float3 vAmbient = float3(0.0, 0.0, 0.0);
#endif
	return float4(vAlbedo.rgb * saturate(vDiffuse + vAmbient), 1);
}

)";
//!--------------------------------- EmbeddedShaders/PrimitiveBatchShader.hlsl ---------------------------------
const char* HLSL_PrimitiveBatchShader = R"(

PLAY3D_PER_FRAME_CONSTANTS

struct VSInput
{
	float3 position : POSITION;
	float4 colour : COLOUR;
};

struct PSInput
{
	float4 position : SV_POSITION;
	float4 colour : COLOUR;
};

PSInput VS_Main(VSInput input)
{
	PSInput output;

	output.position = mul(viewProjectionMtx, float4(input.position.xyz, 1.0f));
	output.colour = pow(input.colour, 2.2);

	return output;
}

float4 PS_Main(PSInput input) : SV_TARGET
{
	return input.colour;
}

)";
//!----------------------------------- EmbeddedShaders/SkinnedMeshShader.hlsl -----------------------------------
const char* HLSL_SkinnedMeshShader = R"(
PLAY3D_SKINNED_MESH_VS_INPUTS

struct PSInput
{
	float4 position : SV_POSITION;
};

PSInput VS_Main(VSInput input)
{
	PSInput output;
	output.position = float4(0, 0, 0, 1);
	return output;
}

)";
//!----------------------------------- EmbeddedShaders/SpriteBatchShader.hlsl -----------------------------------
const char* HLSL_SpriteBatchShader = R"(

PLAY3D_PER_FRAME_CONSTANTS
PLAY3D_PER_DRAW_CONSTANTS

struct SpriteInfo
{
	uint4 dimPixels;
	float4 uv[4];
	float4 offset;
};

StructuredBuffer<SpriteInfo> g_atlas : register(t4);

struct VSInput
{
	uint vertexId : SV_VertexID;

	uint instanceId : SV_InstanceID;
	float4 posRotate : POSROTATE;
	float4 scaleOffset : SCALEOFFSET;
	float4 colour : COLOUR;
	uint4 index : INDEX;
};

struct PSInput
{
	float4 position : SV_POSITION;
	float4 colour : COLOUR;
	float2 uv : UV;
};

static const float2 g_quadOffset[4] = {float2(0, 0), float2(1, 0), float2(0, 1), float2(1, 1)};

PSInput VS_Main(VSInput input)
{
	PSInput output;

	uint atlasIndex = input.index.x;

	float2 pos_ls = (g_quadOffset[input.vertexId] - g_atlas[atlasIndex].offset.xy) * float2(g_atlas[atlasIndex].dimPixels.zw) * input.scaleOffset.xy + input.scaleOffset.zw;

	float c = cos(input.posRotate.w);
	float s = sin(input.posRotate.w);
	float2x2 rotMtx = float2x2(
		c, -s,
		s, c);

	output.position = mul(worldMtx, float4(input.posRotate.xyz + float3(mul(rotMtx, pos_ls), 0.f), 1.0f));
	output.colour = pow(input.colour, 2.2);

	output.uv = g_atlas[atlasIndex].uv[input.vertexId].xy;
	return output;
}

Texture2D g_colourTexture : register(t0);
SamplerState g_linearSampler : register(s0);

float4 PS_Main(PSInput input)
	: SV_TARGET
{
	float4 Cd = g_colourTexture.Sample(g_linearSampler, input.uv);
	clip(Cd.a);
	return float4(Cd.rgb * input.colour.rgb, Cd.a);
}

)";
//!------------------------------------ EmbeddedShaders/StandardMacros.hlsl ------------------------------------
const char* HLSL_StandardMacros = R"(

#define PLAY3D_PER_FRAME_CONSTANTS\
cbuffer                                                                                   \
	FrameConstantData:                                                                                                 \
	register(b0)                                                                                                       \
	{                                                                                                                  \
		float4x4 viewMtx;                                                                                              \
		float4x4 projectionMtx;                                                                                        \
		float4x4 viewProjectionMtx;                                                                                    \
		float4x4 invViewMtx;                                                                                           \
		float4x4 invProjectionMtx;                                                                                     \
		float4 viewPosition;                                                                                           \
		float4 time;                                                                                                   \
	};

#define PLAY3D_PER_DRAW_CONSTANTS\
cbuffer                                                                                    \
	DrawConstantData:                                                                                                  \
	register(b1)                                                                                                       \
	{                                                                                                                  \
		float4x4 mvpMtx;                                                                                               \
		float4x4 worldMtx;                                                                                             \
		float4x4 normalMtx;                                                                                            \
	};

#define MAX_LIGHTS 4

#define PLAY3D_LIGHT_CONSTANTS\
cbuffer                                                                                       \
	LightConstantData:                                                                                                 \
	register(b2)                                                                                                       \
	{                                                                                                                  \
		float4 lightPos[MAX_LIGHTS];                                                                                   \
		float4 lightDir[MAX_LIGHTS];                                                                                   \
		float4 lightColour[MAX_LIGHTS];                                                                                \
		float4 lightAmbient;                                                                                           \
	};

#define PLAY3D_MATERIAL_CONSTANTS\
cbuffer                                                                                    \
	MaterialConstantData:                                                                                              \
	register(b3)                                                                                                       \
	{                                                                                                                  \
		float4 diffuseColour;                                                                                          \
		float4 specularColour;                                                                                         \
	};

#define PLAY3D_MESH_VS_INPUTS\
struct                                                                                        \
	VSInput                                                                                                            \
	{                                                                                                                  \
		float3 position : POSITION;                                                                                    \
		float4 colour : COLOUR;                                                                                        \
		float3 normal : NORMAL;                                                                                        \
		float2 uv : UV;                                                                                                \
		float4 tangent : TANGENT;                                                                                      \
	};

#define PLAY3D_SKINNED_MESH_VS_INPUTS\
struct                                                                                \
	VSInput                                                                                                            \
	{                                                                                                                  \
		float3 position : POSITION;                                                                                    \
		float4 colour : COLOUR;                                                                                        \
		float3 normal : NORMAL;                                                                                        \
		float2 uv : UV;                                                                                                \
		float4 tangent : TANGENT;                                                                                      \
		uint4  jointId : JOINTID;                                                                                        \
		float4 jointWeight : JOINTWEIGHT;                                                                                  \
	};

)";
}

//-------------------------------------------- Graphics/Texture.cpp --------------------------------------------

namespace Play3d::Graphics
{
	DXGI_FORMAT TranslateTextureFormat(TextureFormat format)
	{
		switch (format)
		{
		case TextureFormat::GRAYSCALE: return DXGI_FORMAT_R8_UNORM;
		case TextureFormat::RGBA: return DXGI_FORMAT_R8G8B8A8_UNORM_SRGB;
		case TextureFormat::BGRA: return DXGI_FORMAT_B8G8R8A8_UNORM_SRGB;
		case TextureFormat::RGBA_F16: return DXGI_FORMAT_R16G16B16A16_FLOAT;
		case TextureFormat::DEPTH: return DXGI_FORMAT_R32_TYPELESS;

		default: return DXGI_FORMAT_R8_UNORM;
		};
	}

	u32 TexturePitchByFormat(TextureFormat format)
	{
		switch (format)
		{
		case TextureFormat::GRAYSCALE: return 1;
		case TextureFormat::RGBA: return 4;
		case TextureFormat::RGBA_F16: return 8;
		case TextureFormat::DEPTH: return 4;
		default: return 4;
		};
	}

	Texture::Texture(const TextureDesc& rDesc)
	{
		bool bIsDepthFormat = rDesc.m_format == TextureFormat::DEPTH;
		bool bIsTexture = (rDesc.m_flags & TextureFlags::ENABLE_TEXTURE);
		bool bIsRenderSurface = (rDesc.m_flags & TextureFlags::ENABLE_RENDER_TARGET);

		ID3D11Device* pDevice = Graphics::Graphics_Impl::Instance().GetDevice();

		D3D11_TEXTURE2D_DESC desc = {};
		desc.Width = rDesc.m_width;
		desc.Height = rDesc.m_height;
		desc.ArraySize = 1;
		desc.Format = TranslateTextureFormat(rDesc.m_format);
		desc.SampleDesc.Count = 1;
		desc.SampleDesc.Quality = 0;
		desc.CPUAccessFlags = 0;
		desc.BindFlags = bIsTexture ? D3D11_BIND_SHADER_RESOURCE : 0;
		desc.MiscFlags = 0;

		if (!bIsDepthFormat)
		{
			if (rDesc.m_flags & TextureFlags::GENERATE_MIPS)
			{
				desc.MipLevels = (u32)log2(std::min(rDesc.m_width, rDesc.m_height));
				desc.Usage = D3D11_USAGE_DEFAULT;
				desc.BindFlags = D3D11_BIND_RENDER_TARGET | D3D11_BIND_SHADER_RESOURCE;
				desc.MiscFlags = D3D11_RESOURCE_MISC_GENERATE_MIPS;
			}
			else
			{
				desc.MipLevels = 1;
				desc.Usage = rDesc.m_pImageData ? D3D11_USAGE_IMMUTABLE : D3D11_USAGE_DEFAULT;
			}
		}

		if (bIsRenderSurface)
		{
			if (bIsDepthFormat)
			{
				desc.BindFlags |= D3D11_BIND_DEPTH_STENCIL;
			}
			else
			{
				desc.BindFlags |= D3D11_BIND_RENDER_TARGET;
			}
		}

		if ((rDesc.m_flags & TextureFlags::ENABLE_DYNAMIC) > 0)
		{
			desc.CPUAccessFlags |= D3D11_CPU_ACCESS_WRITE;
			desc.Usage = D3D11_USAGE_DYNAMIC;
		}

		std::vector<D3D11_SUBRESOURCE_DATA> subresources(desc.MipLevels);

		u32 mipWidth = rDesc.m_width;
		for (u32 i = 0; i < desc.MipLevels; ++i)
		{
			subresources[i].pSysMem = rDesc.m_pImageData;
			subresources[i].SysMemPitch = mipWidth * TexturePitchByFormat(rDesc.m_format);
			mipWidth /= 2;
		}

		HRESULT hr = pDevice->CreateTexture2D(&desc, rDesc.m_pImageData ? subresources.data() : nullptr, &m_pTexture);
		PLAY_ASSERT(SUCCEEDED(hr));
		if (rDesc.m_pDebugName)
		{
			m_pTexture->SetPrivateData(WKPDID_D3DDebugObjectName, (UINT)strlen(rDesc.m_pDebugName), rDesc.m_pDebugName);
		}

		if (bIsTexture)
		{
			if (!bIsDepthFormat)
			{
				hr = pDevice->CreateShaderResourceView(m_pTexture.Get(), nullptr, &m_pSRV);
				PLAY_ASSERT(SUCCEEDED(hr));
			}
			else
			{
				D3D11_SHADER_RESOURCE_VIEW_DESC srvViewDesc = {};
				srvViewDesc.Format = DXGI_FORMAT_R32_FLOAT;
				srvViewDesc.ViewDimension = D3D11_SRV_DIMENSION_TEXTURE2D;
				srvViewDesc.Texture2D.MostDetailedMip = 0;
				srvViewDesc.Texture2D.MipLevels = 1;

				hr = pDevice->CreateShaderResourceView(m_pTexture.Get(), &srvViewDesc, &m_pSRV);
				PLAY_ASSERT(SUCCEEDED(hr));
			}
		}

		if (bIsRenderSurface)
		{
			if (rDesc.m_format != TextureFormat::DEPTH)
			{
				hr = pDevice->CreateRenderTargetView(m_pTexture.Get(), nullptr, &m_pRTV);
				PLAY_ASSERT(SUCCEEDED(hr));
			}
			else
			{
				D3D11_DEPTH_STENCIL_VIEW_DESC dsvViewDesc = {};
				dsvViewDesc.Format = DXGI_FORMAT_D32_FLOAT;
				dsvViewDesc.ViewDimension = D3D11_DSV_DIMENSION_TEXTURE2D;
				dsvViewDesc.Flags = 0;

				hr = pDevice->CreateDepthStencilView(m_pTexture.Get(), &dsvViewDesc, &m_pDSV);
				PLAY_ASSERT(SUCCEEDED(hr));
			}
		}

		if (rDesc.m_flags & TextureFlags::GENERATE_MIPS)
		{
			Graphics::Graphics_Impl::Instance().QueueGenerateMips(m_pSRV.Get());
		}
	}

	Texture::Texture(const TextureArrayDesc& rDesc)
	{
		PLAY_ASSERT((rDesc.m_format != TextureFormat::DEPTH));
		PLAY_ASSERT((rDesc.m_flags & TextureFlags::ENABLE_RENDER_TARGET) == false);

		ID3D11Device* pDevice = Graphics::Graphics_Impl::Instance().GetDevice();

		D3D11_TEXTURE2D_DESC desc = {};
		desc.Width = rDesc.m_width;
		desc.Height = rDesc.m_height;
		desc.ArraySize = (UINT)rDesc.m_ppImageData.size();
		desc.Format = TranslateTextureFormat(rDesc.m_format);
		desc.SampleDesc.Count = 1;
		desc.SampleDesc.Quality = 0;
		desc.CPUAccessFlags = 0;
		desc.BindFlags = D3D11_BIND_SHADER_RESOURCE;
		desc.MiscFlags = 0;

		if (rDesc.m_flags & TextureFlags::GENERATE_MIPS)
		{
			desc.MipLevels = (u32)log2(std::min(rDesc.m_width, rDesc.m_height));
			desc.Usage = D3D11_USAGE_DEFAULT;
			desc.BindFlags = D3D11_BIND_RENDER_TARGET | D3D11_BIND_SHADER_RESOURCE;
			desc.MiscFlags = D3D11_RESOURCE_MISC_GENERATE_MIPS;
		}
		else
		{
			desc.MipLevels = 1;
			desc.Usage = D3D11_USAGE_IMMUTABLE;
		}

		if ((rDesc.m_flags & TextureFlags::ENABLE_DYNAMIC) > 0)
		{
			desc.CPUAccessFlags |= D3D11_CPU_ACCESS_WRITE;
			desc.Usage = D3D11_USAGE_DYNAMIC;
		}

		std::vector<D3D11_SUBRESOURCE_DATA> subresources(desc.MipLevels * desc.ArraySize);

		u32 index = 0;
		for (u32 j = 0; j < desc.ArraySize; ++j)
		{
			u32 mipWidth = rDesc.m_width;
			for (u32 i = 0; i < desc.MipLevels; ++i)
			{
				subresources[index].pSysMem = rDesc.m_ppImageData[j].data();
				subresources[index].SysMemPitch = mipWidth * TexturePitchByFormat(rDesc.m_format);
				mipWidth /= 2;
				index++;
			}
		}

		HRESULT hr = pDevice->CreateTexture2D(&desc, subresources.data(), &m_pTexture);
		PLAY_ASSERT(SUCCEEDED(hr));
		if (rDesc.m_pDebugName)
		{
			m_pTexture->SetPrivateData(WKPDID_D3DDebugObjectName, (UINT)strlen(rDesc.m_pDebugName), rDesc.m_pDebugName);
		}

		hr = pDevice->CreateShaderResourceView(m_pTexture.Get(), nullptr, &m_pSRV);
		PLAY_ASSERT(SUCCEEDED(hr));

		if (rDesc.m_flags & TextureFlags::GENERATE_MIPS)
		{
			Graphics::Graphics_Impl::Instance().QueueGenerateMips(m_pSRV.Get());
		}
	}
}

//--------------------------------------------- Input/InputApi.cpp ---------------------------------------------

//--------------------------------------------- Input/Input_Impl.h ---------------------------------------------

namespace Play3d::Input
{
	class Input_Impl
	{
		PLAY_NONCOPYABLE(Input_Impl);
		PLAY_SINGLETON_INTERFACE(Input_Impl);

		Input_Impl();
		~Input_Impl();

	public:
		void BeginFrame();

		void EndFrame();

		void UpdateKeyboard();

		void UpdateXInput();

		bool IsKeyDown(u32 keycode);

		bool IsKeyUp(u32 keycode);

		bool IsKeyPressed(u32 keycode);

		bool IsPadConnected(u32 padId);

		bool IsPadButtonUp(u32 padId, ButtonId button);

		bool IsPadButtonDown(u32 padId, ButtonId button);

		bool IsPadButtonPressed(u32 padId, ButtonId button);

		f32 GetPadAxis(u32 padId, AxisId axis);

		const MouseState& GetMouseState();

		void CaptureMouse(bool bEnable);

	private:
		void UpdateMouse();

		int HandleWindowMessages(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam);

	private:
		MouseState m_mouseState;

		BYTE* m_pKeyState;
		BYTE* m_pPrevKeyState;

		XINPUT_STATE* m_XInputPrevState;
		XINPUT_STATE* m_XInputState;

		f32* m_XInputDeadZoneMags;
		u32 m_padConnectedFlags;

		POINT m_mouseHidePos;
	};
}

namespace Play3d::Input
{
	bool IsKeyPressed(u32 keycode)
	{
		return Input_Impl::Instance().IsKeyPressed(keycode);
	}

	bool IsKeyDown(u32 keycode)
	{
		return Input_Impl::Instance().IsKeyDown(keycode);
	}

	bool IsButtonPressed(InputDevice device, ButtonId buttonId)
	{
		return Input_Impl::Instance().IsPadButtonPressed((u32)device, buttonId);
	}

	bool IsButtonDown(InputDevice device, ButtonId buttonId)
	{
		return Input_Impl::Instance().IsPadButtonDown((u32)device, buttonId);
	}

	float GetAxis(InputDevice device, AxisId axisId)
	{
		return Input_Impl::Instance().GetPadAxis((u32)device, axisId);
	}

	const MouseState& GetMouseState()
	{
		return Input_Impl::Instance().GetMouseState();
	}

	void CaptureMouse(bool bEnable)
	{
		return Input_Impl::Instance().CaptureMouse(bEnable);
	}
}

//-------------------------------------------- Input/Input_Impl.cpp --------------------------------------------

#pragma comment(lib, "XInput")

namespace Play3d::Input
{
	PLAY_SINGLETON_IMPL(Input_Impl);

	u32 ConvertInputKeyCode(u32 value)
	{
		PLAY_ASSERT(value < 256);
		return value;
	}
	u32 ConvertButtonId(ButtonId value)
	{
		static u32 s_lut[] = {
			XINPUT_GAMEPAD_DPAD_UP,
			XINPUT_GAMEPAD_DPAD_DOWN,
			XINPUT_GAMEPAD_DPAD_LEFT,
			XINPUT_GAMEPAD_DPAD_RIGHT,
			XINPUT_GAMEPAD_X,
			XINPUT_GAMEPAD_Y,
			XINPUT_GAMEPAD_A,
			XINPUT_GAMEPAD_B,
			XINPUT_GAMEPAD_LEFT_SHOULDER,
			XINPUT_GAMEPAD_RIGHT_SHOULDER,
			XINPUT_GAMEPAD_LEFT_THUMB,
			XINPUT_GAMEPAD_RIGHT_THUMB,
			XINPUT_GAMEPAD_START,
			XINPUT_GAMEPAD_BACK,
			0,
			0,
		};
		static_assert(sizeof(s_lut) / sizeof(u32) == (u32)ButtonId::BUTTON_COUNT, "Incorrect lookup table size");

		return s_lut[static_cast<u32>(value)];
	}

	inline float ComputePadDeadZone(float LX, float LY, float deadZone)
	{

		float magnitude = sqrt(LX * LX + LY * LY);

		float normalizedLX = LX / magnitude;
		float normalizedLY = LY / magnitude;

		float normalizedMagnitude = 0.f;

		if (magnitude > deadZone)
		{
			if (magnitude > 32767)
				magnitude = 32767;

			magnitude -= deadZone;

			normalizedMagnitude = magnitude / (32767 - deadZone);
		}
		else
		{
			magnitude = 0.f;
			normalizedMagnitude = 0.f;
		}

		return normalizedMagnitude;
	}

	inline f32 XInputTriggerToFloat(BYTE value)
	{
		return (float)value / 255;
	}

	Input_Impl::Input_Impl()
		: m_pKeyState(new BYTE[512])
		, m_pPrevKeyState(new BYTE[256])
		, m_XInputPrevState(new XINPUT_STATE[XUSER_MAX_COUNT])
		, m_XInputState(new XINPUT_STATE[XUSER_MAX_COUNT])
		, m_XInputDeadZoneMags(new float[XUSER_MAX_COUNT * 2])
	{
		memset(m_pKeyState, 0, sizeof(BYTE) * 256);
		memset(m_pPrevKeyState, 0, sizeof(BYTE) * 256);

		Graphics::Graphics_Impl::Instance().RegisterWindowCallback(
			[&](HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam) {
				return this->HandleWindowMessages(hWnd, message, wParam, lParam);
			});

		::GetCursorPos(&m_mouseHidePos);
		CaptureMouse(false);
	}

	Input_Impl::~Input_Impl()
	{
		RAWINPUTDEVICE Rid[1];
		Rid[0].usUsagePage = HID_USAGE_PAGE_GENERIC;
		Rid[0].usUsage = HID_USAGE_GENERIC_MOUSE;
		Rid[0].dwFlags = RIDEV_REMOVE;
		Rid[0].hwndTarget = NULL;
		RegisterRawInputDevices(Rid, 1, sizeof(Rid[0]));

		delete[] m_pKeyState;
		delete[] m_pPrevKeyState;

		m_pKeyState = m_pPrevKeyState = nullptr;

		delete[] m_pPrevKeyState;
		delete[] m_XInputState;
		delete[] m_XInputDeadZoneMags;
	}

	void Input_Impl::BeginFrame()
	{
		UpdateKeyboard();
		UpdateXInput();
	}

	void Input_Impl::EndFrame()
	{
		UpdateMouse();
	}

	bool Input_Impl::IsKeyDown(u32 keycode)
	{
		u32 vkeyCode = ConvertInputKeyCode(keycode);
		PLAY_ASSERT_MSG(vkeyCode < 256, "Unexpected virtual key code");
		return (m_pKeyState[vkeyCode] & 0x80) > 0;
	}

	bool Input_Impl::IsKeyUp(u32 keycode)
	{
		u32 vkeyCode = ConvertInputKeyCode(keycode);
		PLAY_ASSERT_MSG(vkeyCode < 256, "Unexpected virtual key code");
		return (m_pKeyState[vkeyCode] & 0x80) == 0;
	}

	bool Input_Impl::IsKeyPressed(u32 keycode)
	{
		u32 vkeyCode = ConvertInputKeyCode(keycode);
		PLAY_ASSERT_MSG(vkeyCode < 256, "Unexpected virtual key code");
		return ((m_pPrevKeyState[vkeyCode] & 0x80) == 0) && ((m_pKeyState[vkeyCode] & 0x80) > 0);
	}

	bool Input_Impl::IsPadConnected(u32 padId)
	{
		if (padId >= XUSER_MAX_COUNT)
			return false;

		return (m_padConnectedFlags & (1 << padId)) != 0;
	}

	bool Input_Impl::IsPadButtonUp(u32 padId, ButtonId button)
	{
		if (IsPadConnected(padId))
		{
			const XINPUT_STATE& padState(m_XInputState[padId]);

			switch (button)
			{
			case ButtonId::BUTTON_LEFT_TRIGGER: return padState.Gamepad.bLeftTrigger < XINPUT_GAMEPAD_TRIGGER_THRESHOLD;
			case ButtonId::BUTTON_RIGHT_TRIGGER:
				return padState.Gamepad.bRightTrigger < XINPUT_GAMEPAD_TRIGGER_THRESHOLD;
			}

			u32 buttonMask = ConvertButtonId(button);
			return (padState.Gamepad.wButtons & buttonMask) == 0;
		};
		return false;
	}

	bool Input_Impl::IsPadButtonDown(u32 padId, ButtonId button)
	{
		if (IsPadConnected(padId))
		{
			const XINPUT_STATE& padState(m_XInputState[padId]);

			switch (button)
			{
			case ButtonId::BUTTON_LEFT_TRIGGER:
				return padState.Gamepad.bLeftTrigger >= XINPUT_GAMEPAD_TRIGGER_THRESHOLD;
			case ButtonId::BUTTON_RIGHT_TRIGGER:
				return padState.Gamepad.bRightTrigger >= XINPUT_GAMEPAD_TRIGGER_THRESHOLD;
			}

			u32 buttonMask = ConvertButtonId(button);
			return (padState.Gamepad.wButtons & buttonMask) != 0;
		};
		return false;
	}

	bool Input_Impl::IsPadButtonPressed(u32 padId, ButtonId button)
	{
		if (IsPadConnected(padId))
		{
			const XINPUT_STATE& prevPadState(m_XInputPrevState[padId]);
			const XINPUT_STATE& padState(m_XInputState[padId]);

			switch (button)
			{
			case ButtonId::BUTTON_LEFT_TRIGGER:
				return padState.Gamepad.bLeftTrigger > XINPUT_GAMEPAD_TRIGGER_THRESHOLD
					   && prevPadState.Gamepad.bLeftTrigger < XINPUT_GAMEPAD_TRIGGER_THRESHOLD;
			case ButtonId::BUTTON_RIGHT_TRIGGER:
				return padState.Gamepad.bRightTrigger > XINPUT_GAMEPAD_TRIGGER_THRESHOLD
					   && prevPadState.Gamepad.bLeftTrigger < XINPUT_GAMEPAD_TRIGGER_THRESHOLD;
			}

			u32 buttonMask = ConvertButtonId(button);
			return (padState.Gamepad.wButtons & buttonMask) != 0 && (prevPadState.Gamepad.wButtons & buttonMask) == 0;
		};
		return false;
	}

	f32 Input_Impl::GetPadAxis(u32 padId, AxisId axis)
	{
		if (IsPadConnected(padId))
		{
			const XINPUT_STATE& padState(m_XInputState[padId]);

			switch (axis)
			{
			case AxisId::AXIS_LEFTSTICK_X:
				return (f32)padState.Gamepad.sThumbLX / 0x7FFF * m_XInputDeadZoneMags[padId * 2];
			case AxisId::AXIS_LEFTSTICK_Y:
				return (f32)padState.Gamepad.sThumbLY / 0x7FFF * m_XInputDeadZoneMags[padId * 2];
			case AxisId::AXIS_RIGHTSTICK_X:
				return (f32)padState.Gamepad.sThumbRX / 0x7FFF * m_XInputDeadZoneMags[padId * 2 + 1];
			case AxisId::AXIS_RIGHTSTICK_Y:
				return (f32)padState.Gamepad.sThumbRY / 0x7FFF * m_XInputDeadZoneMags[padId * 2 + 1];

			case AxisId::AXIS_LEFT_TRIGGER_PRESSURE: return XInputTriggerToFloat(padState.Gamepad.bLeftTrigger);
			case AxisId::AXIS_RIGHT_TRIGGER_PRESSURE: return XInputTriggerToFloat(padState.Gamepad.bRightTrigger);
			default: return 0.f;
			}
		};
		return false;
	}

	const MouseState& Input_Impl::GetMouseState()
	{
		return m_mouseState;
	}

	void Input_Impl::CaptureMouse(bool bEnable)
	{
		HWND hWnd = Graphics::Graphics_Impl::Instance().GetHWnd();
		if (bEnable)
		{
			int ret;
			do
			{
				ret = ::ShowCursor(FALSE);
			} while (ret > -1);

			RAWINPUTDEVICE Rid[1];
			Rid[0].usUsagePage = HID_USAGE_PAGE_GENERIC;
			Rid[0].usUsage = HID_USAGE_GENERIC_MOUSE;
			Rid[0].dwFlags = RIDEV_NOLEGACY | RIDEV_INPUTSINK;
			Rid[0].hwndTarget = hWnd;
			RegisterRawInputDevices(Rid, 1, sizeof(Rid[0]));

			::GetCursorPos(&m_mouseHidePos);
		}
		else
		{
			RAWINPUTDEVICE Rid[1];
			Rid[0].usUsagePage = HID_USAGE_PAGE_GENERIC;
			Rid[0].usUsage = HID_USAGE_GENERIC_MOUSE;
			Rid[0].dwFlags = 0;
			Rid[0].hwndTarget = hWnd;
			RegisterRawInputDevices(Rid, 1, sizeof(Rid[0]));

			::SetCursorPos(m_mouseHidePos.x, m_mouseHidePos.y);

			::ShowCursor(TRUE);
		}
	}

	void Input_Impl::UpdateXInput()
	{
		DWORD result;

		m_padConnectedFlags = 0;
		memset(m_XInputDeadZoneMags, 0, sizeof(float) * XUSER_MAX_COUNT * 2);

		std::swap(m_XInputPrevState, m_XInputState);

		for (DWORD i = 0; i < XUSER_MAX_COUNT; i++)
		{

			ZeroMemory(&m_XInputState[i], sizeof(XINPUT_STATE));
			result = XInputGetState(i, &m_XInputState[i]);

			if (ERROR_SUCCESS == result)
			{
				m_padConnectedFlags |= 1 << i;
				m_XInputDeadZoneMags[i * 2] = ComputePadDeadZone(m_XInputState[i].Gamepad.sThumbLX,
																 m_XInputState[i].Gamepad.sThumbLY,
																 XINPUT_GAMEPAD_LEFT_THUMB_DEADZONE);
				m_XInputDeadZoneMags[i * 2 + 1] = ComputePadDeadZone(m_XInputState[i].Gamepad.sThumbRX,
																	 m_XInputState[i].Gamepad.sThumbRY,
																	 XINPUT_GAMEPAD_RIGHT_THUMB_DEADZONE);
			}
		}
	}

	void Input_Impl::UpdateKeyboard()
	{
		std::swap(m_pKeyState, m_pPrevKeyState);
		GetKeyboardState(m_pKeyState);
	}

	void Input_Impl::UpdateMouse()
	{
		m_mouseState.m_deltaX = 0.f;
		m_mouseState.m_deltaY = 0.f;
	}

	int Input_Impl::HandleWindowMessages(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
	{
		switch (message)
		{
		case WM_INPUT: {

			if (GET_RAWINPUT_CODE_WPARAM(wParam) == RIM_INPUT)
			{
				constexpr u32 kInputBufferSize = 64;
				UINT dwSize = kInputBufferSize;
				BYTE lpb[kInputBufferSize];

				GetRawInputData((HRAWINPUT)lParam, RID_INPUT, NULL, &dwSize, sizeof(RAWINPUTHEADER));

				if (dwSize <= kInputBufferSize)
				{
					GetRawInputData((HRAWINPUT)lParam, RID_INPUT, lpb, &dwSize, sizeof(RAWINPUTHEADER));
					RAWINPUT* raw = (RAWINPUT*)lpb;

					if (raw->header.dwType == RIM_TYPEMOUSE)
					{
						const RAWMOUSE& rMouse(raw->data.mouse);
						if ((rMouse.usButtonFlags & RI_MOUSE_LEFT_BUTTON_DOWN) > 0)
						{
							m_mouseState.m_leftButton = true;
						}

						if ((rMouse.usButtonFlags & RI_MOUSE_LEFT_BUTTON_UP) > 0)
						{
							m_mouseState.m_leftButton = false;
						}

						if ((rMouse.usButtonFlags & RI_MOUSE_RIGHT_BUTTON_DOWN) > 0)
						{
							m_mouseState.m_rightButton = true;
						}

						if ((rMouse.usButtonFlags & RI_MOUSE_RIGHT_BUTTON_UP) > 0)
						{
							m_mouseState.m_rightButton = false;
						}

						if (((rMouse.usFlags & MOUSE_MOVE_ABSOLUTE) == MOUSE_MOVE_ABSOLUTE)
							&& ((rMouse.usFlags & MOUSE_VIRTUAL_DESKTOP) == MOUSE_VIRTUAL_DESKTOP))
						{
							static constexpr f32 kVirtualMouseVelocity = 0.5f;
							static LONG s_prevX = rMouse.lLastX;
							static LONG s_prevY = rMouse.lLastY;

							const LONG dx = s_prevX - rMouse.lLastX;
							const LONG dy = s_prevY - rMouse.lLastY;

							const int screenX = GetSystemMetrics(SM_CXVIRTUALSCREEN);
							const int screenY = GetSystemMetrics(SM_CYVIRTUALSCREEN);

							m_mouseState.m_deltaX += -(f32)dx / 65535.0f * screenX * kVirtualMouseVelocity;
							m_mouseState.m_deltaY += -(f32)dy / 65535.0f * screenY * kVirtualMouseVelocity;

							s_prevX = rMouse.lLastX;
							s_prevY = rMouse.lLastY;
						}
						else
						{
							m_mouseState.m_deltaX += rMouse.lLastX;
							m_mouseState.m_deltaY += rMouse.lLastY;
						}
					}
				}
			}

			break;
		}

		case WM_MOUSEMOVE:
			m_mouseState.m_x = (f32)LOWORD(lParam);
			m_mouseState.m_y = (f32)HIWORD(lParam);
			break;
		};

		return 0;
	}
}

//----------------------------------------- Maths/IntersectionMath.cpp -----------------------------------------

namespace Play3d
{

	void ComputeBoundsFromPositions(const Vector3f* pPositions, size_t numElements, AABBMinMax& rBoundsOut)
	{
		PLAY_ASSERT(numElements > 0);

		rBoundsOut.vMin = rBoundsOut.vMax = pPositions[0];

		for (size_t i = 1; i < numElements; ++i)
		{
			const Vector3f& p(pPositions[i]);

			if (p.x < rBoundsOut.vMin.x)
				rBoundsOut.vMin.x = p.x;
			if (p.y < rBoundsOut.vMin.y)
				rBoundsOut.vMin.y = p.y;
			if (p.z < rBoundsOut.vMin.z)
				rBoundsOut.vMin.z = p.z;

			if (p.x > rBoundsOut.vMax.x)
				rBoundsOut.vMax.x = p.x;
			if (p.y > rBoundsOut.vMax.y)
				rBoundsOut.vMax.y = p.y;
			if (p.z > rBoundsOut.vMax.z)
				rBoundsOut.vMax.z = p.z;
		}
	}

}

//----------------------------------------------- Play3dApi.cpp -----------------------------------------------

//----------------------------------------- Resources/ResourcesApi.cpp -----------------------------------------

namespace Play3d::Resources
{
	IdKey<AsyncLoadingTask> AsyncLoadAssets(std::string_view pathToAssetPack)
	{
		return IdKey<AsyncLoadingTask>();
	}

	bool AssetsLoaded(IdKey<AsyncLoadingTask> hAsyncLoad)
	{
		return true;
	}

	Play3d::result_t LoadAssets(std::string_view pathToAssetPack)
	{
		return 0;
	}
}

//-------------------------------------------- Sprite/SpriteApi.cpp --------------------------------------------

//----------------------------------------- Sprite/SpriteBatch_Impl.h -----------------------------------------

namespace Play3d::Sprite
{
	struct SpriteVertex
	{
		Vector4f posRotate;
		Vector4f scaleOffset;
		ColourValue colour[4];
		u32 index[4];
	};
	static_assert(sizeof(SpriteVertex) % 16 == 0);

	class SpriteBatch
	{
		friend class SpriteRenderSystem;
	public:
		SpriteBatch(ID3D11Device* pDevice, u32 kMaxVertexCount);
		~SpriteBatch();

		void Setup(SpriteAtlasId hAtlas, Graphics::MaterialId materialId, const Matrix4x4f& viewProject,
				   SpriteBatchSortOrder sortOrder);

		void AppendSprite(const SpriteVertex& v);

		void Flush(ID3D11DeviceContext* pContext);

		void Bind(ID3D11DeviceContext* pContext);

		void DrawSprites(ID3D11DeviceContext* pContext);

	private:
		std::vector<SpriteVertex> m_vertices;

		ComPtr<ID3D11Buffer> m_pVertexBuffer;

		u32 m_maxVertexCount;
		u32 m_instanceCount;

		SpriteAtlasId m_hAtlas;
		Graphics::MaterialId m_materialId;
		Matrix4x4f m_viewProject;
		SpriteBatchSortOrder m_sortOrder;
	};

	//! @brief Manages the allocation and rendering of sprite batches.
	class SpriteRenderSystem : public Graphics::IGraphicsCallbacks
	{
		PLAY_SINGLETON_INTERFACE(SpriteRenderSystem);

	public:
		SpriteRenderSystem();
		~SpriteRenderSystem();
		void OnBeginFrame() override;
		void OnEndFrame() override;

		SpriteBatch* AllocateSpriteBatch();
		void DrawSpriteBatch(SpriteBatch* pBatch);

		Graphics::ShaderId GetSpriteVS() { return m_spriteBatchVS; }
		Graphics::ShaderId GetSpritePS() { return m_spriteBatchPS; }

	private:
		std::vector<SpriteBatch*> m_spriteBatchRing;
		ComPtr<ID3D11InputLayout> m_pSpriteInputLayout;

		ComPtr<ID3D11RasterizerState> m_pRasterState;
		ComPtr<ID3D11DepthStencilState> m_pDepthStencilState;
		ComPtr<ID3D11BlendState> m_pBlendState;

		Graphics::ShaderId m_spriteBatchVS;
		Graphics::ShaderId m_spriteBatchPS;
		u32 m_nNextSpriteBatch;
	};

}

namespace Play3d::Sprite
{
	struct InternalState
	{
		SpriteBatch* m_pCurrentSpriteBatch;
	};

	static InternalState s_internalState;

	SpriteAtlas::SpriteAtlas(const GridSpriteAtlasDesc& rDesc)
		: m_spriteInfo(rDesc.gridSizeX * rDesc.gridSizeY)
	{
		u32 tileWidth = rDesc.imageWidth / rDesc.gridSizeX;
		u32 tileHeight = rDesc.imageHeight / rDesc.gridSizeY;

		LooseSpriteAtlasDesc looseDesc;
		looseDesc.imageWidth = rDesc.imageWidth;
		looseDesc.imageHeight = rDesc.imageHeight;
		looseDesc.spriteCount = rDesc.gridSizeX * rDesc.gridSizeY;
		std::vector<UVRect> uvRectArray(looseDesc.spriteCount);
		looseDesc.uvArray = uvRectArray.data();

		for (u32 iY = 0; iY < rDesc.gridSizeY; ++iY)
		{
			for (u32 iX = 0; iX < rDesc.gridSizeX; ++iX)
			{
				u32 i = iY * rDesc.gridSizeX + iX;

				u32 posX = iX * tileWidth;
				u32 posY = iY * tileHeight;

				uvRectArray[i].x = posX;
				uvRectArray[i].y = posY;
				uvRectArray[i].width = tileWidth;
				uvRectArray[i].height = tileHeight;
			}
		}

		InternalBuild(looseDesc);
	}

	SpriteAtlas::SpriteAtlas(const LooseSpriteAtlasDesc& rDesc)
	{
		InternalBuild(rDesc);
	}

	void SpriteAtlas::InternalBuild(const LooseSpriteAtlasDesc& rDesc)
	{
		Vector4f vRecip(1.f / (f32)rDesc.imageWidth, 1.f / (f32)rDesc.imageHeight, 1.f, 1.f);

		for (u32 i = 0; i < rDesc.spriteCount; ++i)
		{
			const UVRect& t(rDesc.uvArray[i]);
			SpriteInfo& rInfo(m_spriteInfo[i]);

			rInfo.dimPixels = t;
			rInfo.uv[0] = Vector4f((f32)t.x, (f32)t.y, 0.f, 0.f) * vRecip;
			rInfo.uv[1] = Vector4f((f32)t.x + (f32)t.width, (f32)t.y, 0.f, 0.f) * vRecip;
			rInfo.uv[2] = Vector4f((f32)t.x, (f32)t.y + (f32)t.height, 0.f, 0.f) * vRecip;
			rInfo.uv[3] = Vector4f((f32)t.x + (f32)t.width, (f32)t.y + (f32)t.height, 0.f, 0.f) * vRecip;

			rInfo.offset = Vector4f(0.5f, 0.5f, 0.f, 0.f);
		}
		UpdateGPUBuffer();
	}

	void SpriteAtlas::UpdateGPUBuffer()
	{
		if (m_bufferId.IsInvalid())
		{
			Graphics::BufferDesc desc;
			desc.m_bindFlags = Graphics::BufferBindFlags::SRV;
			desc.m_flags = Graphics::BufferFlags::STRUCTURED;
			desc.m_structureStrideBytes = sizeof(SpriteInfo);
			desc.m_pInitialData = m_spriteInfo.data();
			desc.m_sizeBytes = m_spriteInfo.size() * sizeof(SpriteInfo);

			m_bufferId = Resources::CreateAsset<Graphics::Buffer>(desc);
		}
	}

	Play3d::Matrix4x4f SpriteScreenMatrix()
	{
		auto displaySize = Graphics::GetDisplaySurfaceSize();
		return MatrixOrthoProjectRH<f32>(0, (f32)displaySize.m_width, (f32)displaySize.m_height, 0, -10, 10);
	}

	void BeginSpriteBatch(SpriteAtlasId hAtlas, Graphics::MaterialId materialId, const Matrix4x4f& viewProject,
						  SpriteBatchSortOrder sortOrder /*= SpriteBatchSortOrder::kNone*/)
	{
		if (!s_internalState.m_pCurrentSpriteBatch)
		{
			s_internalState.m_pCurrentSpriteBatch = SpriteRenderSystem::Instance().AllocateSpriteBatch();
		}

		s_internalState.m_pCurrentSpriteBatch->Setup(hAtlas, materialId, viewProject, sortOrder);
	}

	void DrawSprite(u32 spriteIndex, const Vector2f& vPosition, float fDepth, float fAngle, const Vector2f& vScale,
					const Vector2f& vOffset, ColourValue colour)
	{
		if (s_internalState.m_pCurrentSpriteBatch)
		{
			SpriteVertex v;
			v.posRotate = Vector4f(vPosition.x, vPosition.y, fDepth, fAngle);
			v.scaleOffset = Vector4f(vScale.x, vScale.y, vOffset.x, vOffset.y);

			for (u32 i = 0; i < 4; ++i)
			{
				v.colour[i] = colour;
				v.index[i] = spriteIndex;
			}

			s_internalState.m_pCurrentSpriteBatch->AppendSprite(v);
		}
	}

	void EndSpriteBatch()
	{
		if (s_internalState.m_pCurrentSpriteBatch)
		{
			SpriteRenderSystem::Instance().DrawSpriteBatch(s_internalState.m_pCurrentSpriteBatch);
			s_internalState.m_pCurrentSpriteBatch = nullptr;
		}
	}

	Play3d::Graphics::MaterialId CreateSpriteMaterial(const char* pName, Graphics::TextureId hTexture,
													  bool bPointSample, bool bBlendEnable)
	{

		Graphics::ComplexMaterialDesc desc = {};
		desc.m_state.m_cullMode = Graphics::CullMode::NONE;
		desc.m_state.m_fillMode = Graphics::FillMode::SOLID;
		desc.m_VertexShader = Sprite::SpriteRenderSystem::Instance().GetSpriteVS();
		desc.m_PixelShader = Sprite::SpriteRenderSystem::Instance().GetSpritePS();
		desc.m_texture[0] = hTexture;
		desc.m_sampler[0] = bPointSample ? Graphics::CreatePointSampler() : Graphics::CreateLinearSampler();
		desc.m_state.m_blendEnable = bBlendEnable;
		desc.m_state.m_blendMode = Graphics::BlendMode::ALPHABLEND;
		desc.m_state.m_depthEnable = false;

		return Resources::CreateAsset<Graphics::Material>(desc);
	}

}

//---------------------------------------- Sprite/SpriteBatch_Impl.cpp ----------------------------------------

namespace Play3d::Sprite
{
	PLAY_SINGLETON_IMPL(SpriteRenderSystem);

	SpriteBatch::SpriteBatch(ID3D11Device* pDevice, u32 kMaxVertexCount)
		: m_maxVertexCount(kMaxVertexCount)
		, m_instanceCount(0)
	{
		D3D11_BUFFER_DESC desc = {};
		desc.ByteWidth = sizeof(SpriteVertex) * kMaxVertexCount;
		desc.Usage = D3D11_USAGE::D3D11_USAGE_DYNAMIC;
		desc.BindFlags = D3D11_BIND_FLAG::D3D11_BIND_VERTEX_BUFFER;
		desc.CPUAccessFlags = D3D11_CPU_ACCESS_WRITE;
		desc.MiscFlags = 0;
		desc.StructureByteStride = 0;
		HRESULT hr = pDevice->CreateBuffer(&desc, NULL, &m_pVertexBuffer);
		PLAY_ASSERT_MSG(SUCCEEDED(hr), "CreateBuffer");
	}

	SpriteBatch::~SpriteBatch()
	{}

	void SpriteBatch::AppendSprite(const SpriteVertex& v)
	{
		m_vertices.push_back(v);
	}

	void SpriteBatch::Flush(ID3D11DeviceContext* pContext)
	{
		D3D11_MAPPED_SUBRESOURCE data;
		HRESULT hr = pContext->Map(m_pVertexBuffer.Get(), 0, D3D11_MAP::D3D11_MAP_WRITE_DISCARD, 0, &data);
		if (SUCCEEDED(hr))
		{
			m_instanceCount = (u32)m_vertices.size();

			SpriteVertex* pVertexOut = (SpriteVertex*)data.pData;
			memcpy(pVertexOut, m_vertices.data(), m_instanceCount * sizeof(SpriteVertex));

			pContext->Unmap(m_pVertexBuffer.Get(), 0);

			m_vertices.clear();
		}
	}

	void SpriteBatch::Bind(ID3D11DeviceContext* pContext)
	{
		ID3D11Buffer* buffers[] = {m_pVertexBuffer.Get()};
		UINT strides[] = {sizeof(SpriteVertex)};
		UINT offsets[] = {0};

		pContext->IASetVertexBuffers(0, 1, buffers, strides, offsets);
	}

	void SpriteBatch::DrawSprites(ID3D11DeviceContext* pContext)
	{
		if (m_instanceCount > 0)
		{
			pContext->IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_TRIANGLESTRIP);
			pContext->DrawInstanced(4, m_instanceCount, 0, 0);
			m_instanceCount = 0;
		}
	}

	void SpriteBatch::Setup(SpriteAtlasId hAtlas, Graphics::MaterialId materialId, const Matrix4x4f& viewProject,
							SpriteBatchSortOrder sortOrder)
	{
		m_hAtlas = hAtlas;
		m_materialId = materialId;
		m_viewProject = viewProject;
		m_sortOrder = sortOrder;
	}

	SpriteRenderSystem::SpriteRenderSystem()
		: m_nNextSpriteBatch(0)
	{
		using namespace Graphics;
		HRESULT hr;

		{
			u32 compilationFlags = 0;
#ifdef _DEBUG
			compilationFlags |= ShaderCompilationFlags::DEBUG;
#endif

			ShaderCompilerDesc desc;
			desc.m_name = "HLSL_SpriteBatchShader";
			desc.m_hlslCode = Graphics::HLSL_SpriteBatchShader;
			desc.m_type = ShaderType::VERTEX_SHADER;
			desc.m_entryPoint = "VS_Main";
			desc.m_flags = compilationFlags;

			m_spriteBatchVS = Shader::Compile(desc);

			desc.m_type = ShaderType::PIXEL_SHADER;
			desc.m_entryPoint = "PS_Main";

			m_spriteBatchPS = Shader::Compile(desc);
		}

		auto pDevice = Graphics_Impl::Instance().GetDevice();

		{
			Shader* pVS = Resources::ResourceManager<Shader>::Instance().GetPtr(m_spriteBatchVS);

			D3D11_INPUT_ELEMENT_DESC vertexFormat[] = {
				{"POSROTATE",0,DXGI_FORMAT_R32G32B32A32_FLOAT,0,offsetof(SpriteVertex, posRotate), D3D11_INPUT_PER_INSTANCE_DATA, 1},
				{"SCALEOFFSET",0,DXGI_FORMAT_R32G32B32A32_FLOAT, 0,offsetof(SpriteVertex, scaleOffset),D3D11_INPUT_PER_INSTANCE_DATA,1},
				{"COLOUR",0,DXGI_FORMAT_R8G8B8A8_UNORM, 0,offsetof(SpriteVertex, colour),D3D11_INPUT_PER_INSTANCE_DATA, 1},
				{"INDEX",0,DXGI_FORMAT_R8G8B8A8_UINT, 0,offsetof(SpriteVertex, index),D3D11_INPUT_PER_INSTANCE_DATA, 1},
			};

			hr = pDevice->CreateInputLayout(vertexFormat, 4, pVS->GetByteCode(),pVS->GetByteCodeSize(),&m_pSpriteInputLayout);
			PLAY_ASSERT_MSG(SUCCEEDED(hr), "CreateInputLayout");
		}

		{
			D3D11_RASTERIZER_DESC desc = {};
			desc.FillMode = D3D11_FILL_MODE::D3D11_FILL_SOLID;
			desc.CullMode = D3D11_CULL_MODE::D3D11_CULL_NONE;
			desc.FrontCounterClockwise = FALSE;
			desc.DepthBias = 0;
			desc.DepthBiasClamp = 0.f;
			desc.SlopeScaledDepthBias = 0.f;
			desc.DepthClipEnable = TRUE;
			desc.ScissorEnable = FALSE;
			desc.MultisampleEnable = FALSE;
			desc.AntialiasedLineEnable = FALSE;
			hr = pDevice->CreateRasterizerState(&desc, &m_pRasterState);
			PLAY_ASSERT_MSG(SUCCEEDED(hr), "Invalid RasterState");
		}
		{
			D3D11_DEPTH_STENCIL_DESC desc = {};
			desc.DepthEnable = TRUE;
			desc.DepthWriteMask = D3D11_DEPTH_WRITE_MASK::D3D11_DEPTH_WRITE_MASK_ALL;
			desc.DepthFunc = D3D11_COMPARISON_FUNC::D3D11_COMPARISON_LESS;
			desc.StencilEnable = FALSE;
			hr = pDevice->CreateDepthStencilState(&desc, &m_pDepthStencilState);
			PLAY_ASSERT_MSG(SUCCEEDED(hr), "Invalid DepthStencilState");
		}
		{
			D3D11_BLEND_DESC desc = {};
			desc.AlphaToCoverageEnable = FALSE;
			desc.IndependentBlendEnable = FALSE;
			desc.RenderTarget[0].BlendEnable = FALSE;
			desc.RenderTarget[0].SrcBlend = D3D11_BLEND_SRC_ALPHA;
			desc.RenderTarget[0].DestBlend = D3D11_BLEND_INV_SRC_ALPHA;
			desc.RenderTarget[0].BlendOp = D3D11_BLEND_OP_ADD;
			desc.RenderTarget[0].SrcBlendAlpha = D3D11_BLEND_ONE;
			desc.RenderTarget[0].DestBlendAlpha = D3D11_BLEND_ZERO;
			desc.RenderTarget[0].BlendOpAlpha = D3D11_BLEND_OP_ADD;
			desc.RenderTarget[0].RenderTargetWriteMask = D3D11_COLOR_WRITE_ENABLE_ALL;
			hr = pDevice->CreateBlendState(&desc, &m_pBlendState);
			PLAY_ASSERT_MSG(SUCCEEDED(hr), "Invalid BlendState");
		}
	}

	SpriteRenderSystem::~SpriteRenderSystem()
	{
		for (auto& it : m_spriteBatchRing)
		{
			PLAY_SAFE_DELETE(it);
		}
	}

	void SpriteRenderSystem::OnBeginFrame()
	{
		m_nNextSpriteBatch = 0;
	}

	void SpriteRenderSystem::OnEndFrame()
	{}

	SpriteBatch* SpriteRenderSystem::AllocateSpriteBatch()
	{
		using namespace Graphics;
		SpriteBatch* pBatch = nullptr;
		if (m_nNextSpriteBatch < m_spriteBatchRing.size())
		{
			pBatch = m_spriteBatchRing[m_nNextSpriteBatch];
			++m_nNextSpriteBatch;
		}
		else
		{
			auto pDevice = Graphics_Impl::Instance().GetDevice();
			pBatch = new SpriteBatch(pDevice, 0x10000);
			m_spriteBatchRing.push_back(pBatch);
			++m_nNextSpriteBatch;
		}
		PLAY_ASSERT(pBatch);
		return pBatch;
	}

	void SpriteRenderSystem::DrawSpriteBatch(SpriteBatch* pBatch)
	{
		using namespace Graphics;
		PLAY_ASSERT(pBatch);

		Graphics_Impl::Instance().SetWorldMatrix(pBatch->m_viewProject);
		Graphics_Impl::Instance().UpdateConstantBuffers();
		ID3D11DeviceContext* pDC = Graphics_Impl::Instance().GetDeviceContext();

		pBatch->Flush(pDC);

		Graphics_Impl::Instance().BindFrameConstants();

		Graphics_Impl::Instance().SetMaterial(pBatch->m_materialId);
		Graphics_Impl::Instance().BindActiveMaterial(pDC);

		pDC->IASetInputLayout(m_pSpriteInputLayout.Get());

		pBatch->Bind(pDC);

		constexpr u32 kAtlasBindSlot = Graphics::kGlobalTextureSlotStart;

		SpriteAtlas* pAtlas = Resources::ResourceManager<SpriteAtlas>::Instance().GetPtr(pBatch->m_hAtlas);
		if (pAtlas)
		{
			Buffer* pAtlasBuffer = Resources::ResourceManager<Buffer>::Instance().GetPtr(pAtlas->GetBuffer());
			if (pAtlasBuffer)
			{
				ID3D11ShaderResourceView* srvs[] = {pAtlasBuffer->GetSRV()};
				pDC->VSSetShaderResources(kAtlasBindSlot, 1, srvs);
			}
		}

		pBatch->DrawSprites(pDC);
	}

}

//-------------------------------------------- System/Packfile.cpp --------------------------------------------

namespace Play3d::Packfile
{

	PackedFileManager* PackedFileManager::ms_pInstance = nullptr;

	PackedFileManager::~PackedFileManager()
	{
		for (size_t i = 0; i < m_packedFiles.size(); i++)
		{
			delete m_packedFiles[i];
		}
		m_packedFiles.clear();
	}

	void PackedFileManager::Initialise()
	{
		if (!ms_pInstance)
		{
			ms_pInstance = new PackedFileManager();
		}
	}

	void PackedFileManager::Destroy()
	{
		delete ms_pInstance;
		ms_pInstance = nullptr;
	}

	void PackedFileManager::AddPackFile(const char* pFilePath)
	{
		if (System::CheckFileExistsOnDisk(pFilePath))
		{
			size_t sizeBytes;
			const void* packFileData = System::LoadFileData(pFilePath, sizeBytes);
			PackedFile* pNewFile = new PackedFile(packFileData, sizeBytes);

			m_packedFiles.push_back(pNewFile);
		}
	}

	const void* PackedFileManager::FindFile(const char* pFilePath, size_t& sizeBytes)
	{
		const void* file = nullptr;
		for (auto packedFile : m_packedFiles)
		{
			file = packedFile->GetFile(pFilePath, sizeBytes);
			if (file != nullptr)
				break;
		}

		return file;
	}

	bool PackedFileManager::CheckPointerIsInPack(const void* ptr)
	{
		for (auto packedFile : m_packedFiles)
		{
			if (packedFile->CheckPointerIsInPack(ptr))
				return true;
		}
		return false;
	}

	PackedFile::PackedFile(const void* pData, size_t sizeBytes)
	{
		m_pRawData = pData;
		m_pHeader = static_cast<const PackedFileHeader*>(m_pRawData);
		m_pDataBlock = reinterpret_cast<const uint8_t*>(static_cast<const uint8_t*>(m_pRawData) + m_pHeader->dataBlockOffset);
		m_pDirBlock = reinterpret_cast<const PackedFileEntry*>(static_cast<const uint8_t*>(m_pRawData) + m_pHeader->dirBlockOffset);
		m_totalSizeBytes = sizeBytes;
	}

	PackedFile::~PackedFile()
	{
		System::ReleaseFileData(m_pRawData);
		m_pRawData = nullptr;
		m_pHeader = nullptr;
		m_pDataBlock = nullptr;
		m_pDirBlock = nullptr;
	}

	const void* PackedFile::GetFile(const char* pFilePath, size_t& sizeBytes)
	{
		for (size_t i = 0; i < m_pHeader->dirEntryCount; ++i)
		{
			if (_stricmp(m_pDirBlock[i].path, pFilePath) == 0)
			{
				sizeBytes = m_pDirBlock[i].dataSize;
				return m_pDataBlock + m_pDirBlock[i].offset;
			}
		}

		sizeBytes = 0;
		return nullptr;
	}

	const size_t PackedFile::GetNumberOfFilesInPackFile()
	{
		return m_pHeader->dirEntryCount;
	}

	const PackedFileEntry* PackedFile::GetFileInfoAtIndex(size_t index)
	{
		return &m_pDirBlock[index];
	}

	bool PackedFile::CheckPointerIsInPack(const void* ptr)
	{
		if (ptr == m_pRawData)
			return false;

		uintptr_t blockOffset = (uintptr_t)ptr - (uintptr_t)m_pRawData;

		return (blockOffset < m_totalSizeBytes);
	}

}

//-------------------------------------------- System/SystemApi.cpp --------------------------------------------

namespace Play3d::System
{
	struct SystemImpl
	{
		SystemImpl()
		{
			m_clockFrequency.QuadPart = 0;
			m_currentTimeMicroSeconds = 0;
			m_fElapsedTime = 0.f;
			m_fDeltaTime = 0.f;

			LARGE_INTEGER timestamp;
			QueryPerformanceFrequency(&m_clockFrequency);
			QueryPerformanceCounter(&timestamp);
			m_prevTimeMicroSeconds = (timestamp.QuadPart * 1000000) / m_clockFrequency.QuadPart;
		}

		void BeginFrame()
		{
			LARGE_INTEGER timestamp;
			QueryPerformanceCounter(&timestamp);
			m_currentTimeMicroSeconds = (timestamp.QuadPart * 1000000) / m_clockFrequency.QuadPart;
			m_deltaTimeMicroSeconds = m_currentTimeMicroSeconds - m_prevTimeMicroSeconds;

			m_fDeltaTime = (f32)m_deltaTimeMicroSeconds / 1000000.0f;
			m_fElapsedTime += (f64)m_fDeltaTime;
		}

		void EndFrame() { m_prevTimeMicroSeconds = m_currentTimeMicroSeconds; }

		static SystemImpl* ms_pInstance;

		LARGE_INTEGER m_clockFrequency;
		u64 m_prevTimeMicroSeconds;
		u64 m_currentTimeMicroSeconds;
		u64 m_deltaTimeMicroSeconds;

		f64 m_fElapsedTime;
		f32 m_fDeltaTime;
	};
	SystemImpl* SystemImpl::ms_pInstance = nullptr;

	bool IsInitialised()
	{
		return SystemImpl::ms_pInstance != nullptr;
	}

	result_t Initialise(const SystemDesc& rDesc)
	{
		if (!SystemImpl::ms_pInstance)
		{
			HRESULT hr;
			hr = CoInitializeEx(nullptr, COINIT_MULTITHREADED);
			if (FAILED(hr))
			{
				Debug::Printf("COM Init Failed with hr=0x%x", hr);
				return hr;
			}

			SystemImpl::ms_pInstance = new SystemImpl;

			Packfile::PackedFileManager::Initialise();

			Graphics::Graphics_Impl::Initialise(rDesc);
			Input::Input_Impl::Initialise();
			Audio::Audio_Impl::Initialise();

			Graphics::PrimitiveRenderSystem::Initialise();
			UI::FontRenderSystem::Initialise();
			Sprite::SpriteRenderSystem::Initialise();

			auto& rGraphicsImpl(Graphics::Graphics_Impl::Instance());
			rGraphicsImpl.PostInitialise();
			rGraphicsImpl.RegisterGraphicsCallbacks(&Graphics::PrimitiveRenderSystem::Instance());
			rGraphicsImpl.RegisterGraphicsCallbacks(&UI::FontRenderSystem::Instance());
			rGraphicsImpl.RegisterGraphicsCallbacks(&Sprite::SpriteRenderSystem::Instance());
		}

		return RESULT_OK;
	}

	result_t BeginFrame()
	{
		PLAY_ASSERT(SystemImpl::ms_pInstance);
		SystemImpl::ms_pInstance->BeginFrame();

		result_t result = Graphics::Graphics_Impl::Instance().BeginFrame();
		if (result != RESULT_OK)
		{
			return result;
		}

		Input::Input_Impl::Instance().BeginFrame();
		Audio::Audio_Impl::Instance().BeginFrame();

		return RESULT_OK;
	}

	result_t EndFrame()
	{
		PLAY_ASSERT(SystemImpl::ms_pInstance);
		SystemImpl::ms_pInstance->EndFrame();
		result_t result = Graphics::Graphics_Impl::Instance().EndFrame();
		if (result != RESULT_OK)
		{
			return result;
		}
		Input::Input_Impl::Instance().EndFrame();
		Audio::Audio_Impl::Instance().EndFrame();

		return RESULT_OK;
	}

	result_t Shutdown()
	{
		if (SystemImpl::ms_pInstance)
		{
			Graphics::Graphics_Impl::Instance().Flush();

			Resources::ResourceManager<UI::Font>::Instance().ReleaseAll();
			Resources::ResourceManager<Graphics::Mesh>::Instance().ReleaseAll();
			Resources::ResourceManager<Graphics::Material>::Instance().ReleaseAll();
			Resources::ResourceManager<Graphics::Shader>::Instance().ReleaseAll();
			Resources::ResourceManager<Graphics::Buffer>::Instance().ReleaseAll();
			Resources::ResourceManager<Graphics::Texture>::Instance().ReleaseAll();
			Resources::ResourceManager<Graphics::Sampler>::Instance().ReleaseAll();

			UI::FontRenderSystem::Destroy();
			Sprite::SpriteRenderSystem::Destroy();
			Graphics::PrimitiveRenderSystem::Destroy();
			Audio::Audio_Impl::Destroy();
			Input::Input_Impl::Destroy();
			Graphics::Graphics_Impl::Destroy();

			Packfile::PackedFileManager::Destroy();

			PLAY_SAFE_DELETE(SystemImpl::ms_pInstance);

			CoUninitialize();
		}
		return RESULT_OK;
	}

	f64 GetElapsedTime()
	{
		PLAY_ASSERT(SystemImpl::ms_pInstance);
		return SystemImpl::ms_pInstance->m_fElapsedTime;
	}

	f32 GetDeltaTime()
	{
		PLAY_ASSERT(SystemImpl::ms_pInstance);
		return SystemImpl::ms_pInstance->m_fDeltaTime;
	}

	bool CheckFileExists(const char* filePath)
	{
		size_t sizeOut = 0;
		const void* pMemoryRet = nullptr;

		pMemoryRet = Packfile::PackedFileManager::Instance().FindFile(filePath, sizeOut);

		if (pMemoryRet != nullptr)
			return true;

		if (CheckFileExistsOnDisk(filePath))
			return true;

		return false;
	}

	bool CheckFileExistsOnDisk(const char* filePath)
	{
		DWORD attrib = GetFileAttributesA(filePath);
		return (attrib != INVALID_FILE_ATTRIBUTES && !(attrib & FILE_ATTRIBUTE_DIRECTORY));
	}

	const void* LoadFileData(const char* filePath, size_t& sizeOut)
	{
		sizeOut = 0;
		const void* pMemoryRet = nullptr;

		pMemoryRet = Packfile::PackedFileManager::Instance().FindFile(filePath, sizeOut);

		if (pMemoryRet != nullptr)
		{
			Debug::Printf("System::LoadFileData: From Pack: %s\n", filePath);
			return pMemoryRet;
		}

		Debug::Printf("System::LoadFileData: From Disk: %s\n", filePath);

		if (!CheckFileExistsOnDisk(filePath))
		{
			Debug::Printf("ERROR: File does not exist! path='%s'\n", filePath);
			return nullptr;
		}

		HANDLE hFile =
			CreateFileA(filePath, GENERIC_READ, FILE_SHARE_READ | FILE_SHARE_WRITE, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);
		if (hFile != INVALID_HANDLE_VALUE)
		{
			LARGE_INTEGER size;
			if (GetFileSizeEx(hFile, &size))
			{
				constexpr u32 kPadding = 16;
				size_t allocSize = (size_t)size.LowPart + kPadding;
				u8* pBuffer = (u8*)_aligned_malloc(allocSize, 16);
				PLAY_ASSERT(pBuffer);

				pBuffer[0] = '\0';
				memset(pBuffer + size.LowPart, 0, kPadding);

				DWORD bytesRead = 0;
				if (ReadFile(hFile, pBuffer, size.LowPart, &bytesRead, NULL))
				{
					sizeOut = size.QuadPart;
					pMemoryRet = pBuffer;
				}
				else
				{
					ReleaseFileData(pBuffer);
				}
			}
			CloseHandle(hFile);
		}
		return pMemoryRet;
	}

	void ReleaseFileData(const void* pMemory)
	{
		if (Packfile::PackedFileManager::Instance().CheckPointerIsInPack(pMemory))
		{
			return;
		}

		if (pMemory)
		{
			_aligned_free(const_cast<void*>(pMemory));
		}
	}
	void AddPackFile(const char* filePath)
	{

		Packfile::PackedFileManager::Instance().AddPackFile(filePath);
	}
}

//! This simplifies the WinMain entry point function as PlayMain()
//! If you want to define your own WinMain then define PLAY_USE_CLIENT_MAIN
#ifndef PLAY_USE_CLIENT_MAIN
extern int PlayMain();
int WINAPI WinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance, LPSTR pCmdLine, int nCmdShow)
{
	return PlayMain();
}
#endif
//------------------------------------------------ TypesApi.cpp ------------------------------------------------

//------------------------------------------------ UI/Font.cpp ------------------------------------------------

namespace Play3d::UI
{
	PLAY_SINGLETON_IMPL(FontRenderSystem);

	Font::Font(const FontDesc& rDesc)
	{
		HANDLE hFontResource = NULL;

		if (!rDesc.m_fontPath.empty())
		{
			size_t sizeBytes = 0;
			const void* pFontFileData = System::LoadFileData(rDesc.m_fontPath.c_str(), sizeBytes);
			if (pFontFileData && sizeBytes)
			{
				DWORD numFontsAdded;

				hFontResource = AddFontMemResourceEx(const_cast<PVOID>(pFontFileData), (DWORD)sizeBytes, 0, &numFontsAdded);
				if (numFontsAdded > 0)
				{
					Debug::Printf("Font file %s added %u fonts\n", rDesc.m_fontPath.c_str(), numFontsAdded);
				}
				else
				{
					Debug::Printf("Font file %s failed to load!\n", rDesc.m_fontPath.c_str());
				}

				System::ReleaseFileData(pFontFileData);
				pFontFileData = nullptr;
			}
		}

		HDC hdc = CreateCompatibleDC(NULL);
		HBITMAP hBitmap = CreateCompatibleBitmap(hdc, 1, 1);
		SelectObject(hdc, hBitmap);

		HFONT hFont = CreateFontA(rDesc.m_pointSize,
								  0,
								  0,
								  0,
								  FW_DONTCARE,
								  FALSE,
								  TRUE,
								  FALSE,
								  ANSI_CHARSET,
								  OUT_OUTLINE_PRECIS,
								  CLIP_DEFAULT_PRECIS,
								  CLEARTYPE_QUALITY,
								  VARIABLE_PITCH,
								  rDesc.m_fontName.c_str());

		SelectObject(hdc, hFont);

		u32 nGlyphs = (u32)rDesc.m_charSet.size();
		PLAY_ASSERT(0 < nGlyphs && nGlyphs < 256);

		memset(m_glyphMap, 0xff, 256);
		for (u32 i = 0; i < nGlyphs; ++i)
		{
			m_glyphMap[rDesc.m_charSet[i]] = (u8)i;
		}

		constexpr u32 kScratchSizeBytes = 256 * 256;
		u8* pScratchImage = new u8[kScratchSizeBytes];

		u32 imageWidth = rDesc.m_textureWidth;
		u32 imageHeight = rDesc.m_textureHeight;

		u32 imageSizeBytes = imageWidth * imageHeight;
		u8* pImage = new u8[imageSizeBytes];
		memset(pImage, 0, imageSizeBytes);

		GLYPHMETRICS glyphMetrics = {};
		MAT2 mat{{0, 1}, {0, 0}, {0, 0}, {0, 1}};

		constexpr u32 tilePadding = 1;
		u32 tilePosX = tilePadding;
		u32 tilePosY = tilePadding;
		u32 maxY = 0;

		const f32 invWidth = 1.0f / imageWidth;
		const f32 invHeight = 1.0f / imageHeight;

		m_glyphData.resize(nGlyphs);

		m_defaultAdvance = 0;
		u32 validGlyphCount = 0;

		for (u32 glyphIndex = 0; glyphIndex < nGlyphs; ++glyphIndex)
		{
			u32 charInSet = rDesc.m_charSet[glyphIndex];
			u32 result = GetGlyphOutlineA(hdc,
										  charInSet,
										  GGO_GRAY8_BITMAP,
										  &glyphMetrics,
										  kScratchSizeBytes,
										  pScratchImage,
										  &mat);
			if (GDI_ERROR != result)
			{
				u32 rowPitch = (glyphMetrics.gmBlackBoxX + 4 - 1) & ~3;

				if ((tilePosX + rowPitch + tilePadding) >= imageWidth)
				{
					tilePosX = tilePadding;
					tilePosY += maxY + tilePadding;
					maxY = 0;
				}

				if (maxY < glyphMetrics.gmBlackBoxY)
				{
					maxY = glyphMetrics.gmBlackBoxY;
				}

				u8* pRasterPos = pImage + (tilePosY * imageWidth + tilePosX);

				for (u32 i = 0; i < glyphMetrics.gmBlackBoxY; ++i)
				{
					for (u32 j = 0; j < rowPitch; ++j)
					{
						pRasterPos[j] = (u8)((u32)pScratchImage[i * rowPitch + j] * 255 / 64);
					}
					pRasterPos += imageWidth;
				}

				GlyphData& rGlyphData(m_glyphData[glyphIndex]);
				rGlyphData.uv0 = Vector2f(tilePosX * invWidth, tilePosY * invHeight);
				rGlyphData.uv1 = Vector2f((tilePosX + glyphMetrics.gmBlackBoxX) * invWidth,
										  (tilePosY + glyphMetrics.gmBlackBoxY) * invHeight);
				rGlyphData.offset0 =
					Vector2f((f32)glyphMetrics.gmptGlyphOrigin.x, (f32)-glyphMetrics.gmptGlyphOrigin.y);
				rGlyphData.offset1 =
					rGlyphData.offset0 + Vector2f((f32)glyphMetrics.gmBlackBoxX, (f32)glyphMetrics.gmBlackBoxY);
				rGlyphData.xAdvance = glyphMetrics.gmCellIncX;
				tilePosX += rowPitch + tilePadding;

				PLAY_ASSERT_MSG(glyphMetrics.gmCellIncX >= 0, "Unexpected");
				m_defaultAdvance += (u32)glyphMetrics.gmCellIncX;
				++validGlyphCount;
			}
			else
			{
				Debug::Printf("Missing glyph (0x%x)\n", charInSet);
				m_glyphMap[charInSet] = 0xFF;
			}
		}

		if (validGlyphCount)
			m_defaultAdvance /= validGlyphCount;

		ID3D11Device* pDevice = Graphics::Graphics_Impl::Instance().GetDevice();

		{
			D3D11_TEXTURE2D_DESC desc = {};
			desc.Width = imageWidth;
			desc.Height = imageHeight;
			desc.MipLevels = 1;
			desc.ArraySize = 1;
			desc.Format = DXGI_FORMAT_R8_UNORM;
			desc.SampleDesc.Count = 1;
			desc.SampleDesc.Quality = 0;
			desc.Usage = D3D11_USAGE_IMMUTABLE;
			desc.BindFlags = D3D11_BIND_SHADER_RESOURCE;
			desc.CPUAccessFlags = 0;
			desc.MiscFlags = 0;

			D3D11_SUBRESOURCE_DATA data;
			data.pSysMem = pImage;
			data.SysMemPitch = imageWidth;

			HRESULT hr = pDevice->CreateTexture2D(&desc, &data, &m_pTexture);
			PLAY_ASSERT(SUCCEEDED(hr));

			hr = pDevice->CreateShaderResourceView(m_pTexture.Get(), NULL, &m_pSRV);
			PLAY_ASSERT(SUCCEEDED(hr));
		}

		{
			D3D11_SAMPLER_DESC desc = {};
			desc.Filter = D3D11_FILTER_MIN_MAG_MIP_POINT;
			desc.AddressU = D3D11_TEXTURE_ADDRESS_CLAMP;
			desc.AddressV = D3D11_TEXTURE_ADDRESS_CLAMP;
			desc.AddressW = D3D11_TEXTURE_ADDRESS_CLAMP;

			HRESULT hr = pDevice->CreateSamplerState(&desc, &m_pSampler);
			PLAY_ASSERT(SUCCEEDED(hr));
		}

		delete[] pScratchImage;
		delete[] pImage;

		{
			const u32 kMaxVertexCount = 0x10000u;

			D3D11_BUFFER_DESC desc = {};
			desc.ByteWidth = sizeof(FontVertex) * kMaxVertexCount;
			desc.Usage = D3D11_USAGE::D3D11_USAGE_DYNAMIC;
			desc.BindFlags = D3D11_BIND_FLAG::D3D11_BIND_VERTEX_BUFFER;
			desc.CPUAccessFlags = D3D11_CPU_ACCESS_WRITE;
			desc.MiscFlags = 0;
			desc.StructureByteStride = 0;
			HRESULT hr = pDevice->CreateBuffer(&desc, NULL, &m_pVertexBuffer);
			PLAY_ASSERT_MSG(SUCCEEDED(hr), "Create Font VertexBuffer");
		}

		{
			D3D11_RASTERIZER_DESC desc = {};
			desc.CullMode = D3D11_CULL_NONE;
			desc.FillMode = D3D11_FILL_SOLID;
			HRESULT hr = pDevice->CreateRasterizerState(&desc, &m_pRasterState);
			PLAY_ASSERT_MSG(SUCCEEDED(hr), "Create Font Raster State");
		}

		if (hFontResource)
		{
			RemoveFontMemResourceEx(hFontResource);
		}
	}

	Font::~Font()
	{}

	void Font::DrawCharacterArray(ID3D11DeviceContext* pDC, const Vector2f& position, const char* pCharArray,
								  const ColourValue* pColourArray, u32 lines, u32 columns)
	{
		u32 blockSize = lines * columns;

		D3D11_MAPPED_SUBRESOURCE data;
		HRESULT hr = pDC->Map(m_pVertexBuffer.Get(), 0, D3D11_MAP::D3D11_MAP_WRITE_DISCARD, 0, &data);
		u32 vertexCount = 0;
		if (SUCCEEDED(hr))
		{
			FontVertex* pVertex = (FontVertex*)data.pData;

			Vector2f cursor(position);

			for (u32 i = 0; i < lines; ++i)
			{
				for (u32 j = 0; j < columns; ++j)
				{
					u32 index = i * columns + j;
					u8 glyphIndex = m_glyphMap[pCharArray[index]];
					if (0xFF == glyphIndex)
					{
						cursor.x += m_defaultAdvance;
						continue;
					}

					const GlyphData& glyph(m_glyphData[glyphIndex]);

					Vector2f v0 = cursor + glyph.offset0;
					Vector2f v1 = cursor + glyph.offset1;

					pVertex[0].position = Vector2f(v0.x, v0.y);
					pVertex[1].position = Vector2f(v1.x, v0.y);
					pVertex[2].position = Vector2f(v1.x, v1.y);
					pVertex[3].position = Vector2f(v0.x, v0.y);
					pVertex[4].position = Vector2f(v1.x, v1.y);
					pVertex[5].position = Vector2f(v0.x, v1.y);

					pVertex[0].uv = Vector2f(glyph.uv0.x, glyph.uv0.y);
					pVertex[1].uv = Vector2f(glyph.uv1.x, glyph.uv0.y);
					pVertex[2].uv = Vector2f(glyph.uv1.x, glyph.uv1.y);
					pVertex[3].uv = Vector2f(glyph.uv0.x, glyph.uv0.y);
					pVertex[4].uv = Vector2f(glyph.uv1.x, glyph.uv1.y);
					pVertex[5].uv = Vector2f(glyph.uv0.x, glyph.uv1.y);

					ColourValue colour(pColourArray[index]);
					pVertex[0].colour = colour;
					pVertex[1].colour = colour;
					pVertex[2].colour = colour;
					pVertex[3].colour = colour;
					pVertex[4].colour = colour;
					pVertex[5].colour = colour;

					pVertex += 6;

					cursor.x += glyph.xAdvance;

					vertexCount += 6;
				}

				cursor.y += 25;
				cursor.x = position.x;
			}

			pDC->Unmap(m_pVertexBuffer.Get(), 0);
		}

		Draw_Internal(pDC, vertexCount);
	}

	void Font::DrawString(ID3D11DeviceContext* pDC, const Vector2f& position, ColourValue colour, std::string_view text)
	{
		D3D11_MAPPED_SUBRESOURCE data;
		HRESULT hr = pDC->Map(m_pVertexBuffer.Get(), 0, D3D11_MAP::D3D11_MAP_WRITE_DISCARD, 0, &data);
		u32 vertexCount = 0;
		if (SUCCEEDED(hr))
		{
			FontVertex* pVertex = (FontVertex*)data.pData;

			Vector2f cursor(position);

			for (u32 i = 0; i < (u32)text.size(); ++i)
			{
				u8 glyphIndex = m_glyphMap[text[i]];
				if (0xFF == glyphIndex)
				{
					cursor.x += m_defaultAdvance;
					continue;
				}

				const GlyphData& glyph(m_glyphData[glyphIndex]);

				Vector2f v0 = cursor + glyph.offset0;
				Vector2f v1 = cursor + glyph.offset1;

				pVertex[0].position = Vector2f(v0.x, v0.y);
				pVertex[1].position = Vector2f(v1.x, v0.y);
				pVertex[2].position = Vector2f(v1.x, v1.y);
				pVertex[3].position = Vector2f(v0.x, v0.y);
				pVertex[4].position = Vector2f(v1.x, v1.y);
				pVertex[5].position = Vector2f(v0.x, v1.y);

				pVertex[0].uv = Vector2f(glyph.uv0.x, glyph.uv0.y);
				pVertex[1].uv = Vector2f(glyph.uv1.x, glyph.uv0.y);
				pVertex[2].uv = Vector2f(glyph.uv1.x, glyph.uv1.y);
				pVertex[3].uv = Vector2f(glyph.uv0.x, glyph.uv0.y);
				pVertex[4].uv = Vector2f(glyph.uv1.x, glyph.uv1.y);
				pVertex[5].uv = Vector2f(glyph.uv0.x, glyph.uv1.y);

				pVertex[0].colour = colour;
				pVertex[1].colour = colour;
				pVertex[2].colour = colour;
				pVertex[3].colour = colour;
				pVertex[4].colour = colour;
				pVertex[5].colour = colour;

				pVertex += 6;

				cursor.x += glyph.xAdvance;

				vertexCount += 6;
			}

			pDC->Unmap(m_pVertexBuffer.Get(), 0);
		}

		Draw_Internal(pDC, vertexCount);
	}

	void Font::Draw_Internal(ID3D11DeviceContext* pDC, u32 vertexCount)
		{
		FontRenderSystem::Instance().PrepFontDraw();

		pDC->IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST);

		ID3D11Buffer* buffers[] = {m_pVertexBuffer.Get()};
		UINT strides[] = {sizeof(FontVertex)};
		UINT offsets[] = {0};
		pDC->IASetVertexBuffers(0, 1, buffers, strides, offsets);

		ID3D11ShaderResourceView* srvs[] = {m_pSRV.Get()};
		pDC->PSSetShaderResources(0, 1, srvs);

		ID3D11SamplerState* samplers[] = {m_pSampler.Get()};
		pDC->PSSetSamplers(0, 1, samplers);

		pDC->RSSetState(m_pRasterState.Get());

		pDC->Draw(vertexCount, 0);
	}

	//! @brief Manages the shared font rendering code for all fonts.
	FontRenderSystem::FontRenderSystem()
	{
		using namespace Graphics;
		HRESULT hr;

		{
			u32 compilationFlags = 0;
#ifdef _DEBUG
			compilationFlags |= ShaderCompilationFlags::DEBUG;
#endif

			ShaderCompilerDesc desc;
			desc.m_name = "HLSL_FontShader";
			desc.m_hlslCode = HLSL_FontShader;
			desc.m_type = ShaderType::VERTEX_SHADER;
			desc.m_entryPoint = "VS_Main";
			desc.m_flags = compilationFlags;

			m_fontVS = Shader::Compile(desc);

			desc.m_type = ShaderType::PIXEL_SHADER;
			desc.m_entryPoint = "PS_Main";

			m_fontPS = Shader::Compile(desc);
		}

		auto pDevice = Graphics_Impl::Instance().GetDevice();

		{
			Shader* pVS = Resources::ResourceManager<Shader>::Instance().GetPtr(m_fontVS);

				D3D11_INPUT_ELEMENT_DESC vertexFormat[] = {
					{"POSITION", 0, DXGI_FORMAT_R32G32_FLOAT, 0, offsetof(UI::FontVertex, position), D3D11_INPUT_PER_VERTEX_DATA, 0},
					{"UV", 0, DXGI_FORMAT_R32G32_FLOAT, 0, offsetof(UI::FontVertex, uv), D3D11_INPUT_PER_VERTEX_DATA, 0},
					{"COLOUR", 0, DXGI_FORMAT_R8G8B8A8_UNORM, 0, offsetof(UI::FontVertex, colour), D3D11_INPUT_PER_VERTEX_DATA, 0},
				};

				hr = pDevice->CreateInputLayout(vertexFormat, 3, pVS->GetByteCode(), pVS->GetByteCodeSize(), &m_pFontInputLayout);
				PLAY_ASSERT_MSG(SUCCEEDED(hr), "CreateInputLayout");
		}

		{
			D3D11_DEPTH_STENCIL_DESC desc = {};
			desc.DepthEnable = FALSE;
			desc.DepthWriteMask = D3D11_DEPTH_WRITE_MASK::D3D11_DEPTH_WRITE_MASK_ALL;
			desc.StencilEnable = FALSE;
			hr = pDevice->CreateDepthStencilState(&desc, &m_pDepthStencilState);
			PLAY_ASSERT_MSG(SUCCEEDED(hr), "Invalid DepthStencilState");
		}
		{
			D3D11_BLEND_DESC desc = {};
			desc.AlphaToCoverageEnable = FALSE;
			desc.IndependentBlendEnable = FALSE;
			desc.RenderTarget[0].BlendEnable = TRUE;
			desc.RenderTarget[0].SrcBlend = D3D11_BLEND_SRC_ALPHA;
			desc.RenderTarget[0].DestBlend = D3D11_BLEND_INV_SRC_ALPHA;
			desc.RenderTarget[0].BlendOp = D3D11_BLEND_OP_ADD;
			desc.RenderTarget[0].SrcBlendAlpha = D3D11_BLEND_ONE;
			desc.RenderTarget[0].DestBlendAlpha = D3D11_BLEND_ZERO;
			desc.RenderTarget[0].BlendOpAlpha = D3D11_BLEND_OP_ADD;
			desc.RenderTarget[0].RenderTargetWriteMask = D3D11_COLOR_WRITE_ENABLE_ALL;
			hr = pDevice->CreateBlendState(&desc, &m_pBlendState);
			PLAY_ASSERT_MSG(SUCCEEDED(hr), "Invalid BlendState");
		}
	}

	FontRenderSystem::~FontRenderSystem()
	{}

	void FontRenderSystem::OnBeginFrame()
	{}

	void FontRenderSystem::OnEndFrame()
	{}

	void FontRenderSystem::PrepFontDraw()
	{
		Graphics::Graphics_Impl::Instance().ClearStateCache();
		Graphics::Graphics_Impl::Instance().UpdateConstantBuffers();

		ID3D11DeviceContext* pDC = Graphics::Graphics_Impl::Instance().GetDeviceContext();

		Graphics::Shader* pVS = Resources::ResourceManager<Graphics::Shader>::Instance().GetPtr(m_fontVS);
		Graphics::Shader* pPS = Resources::ResourceManager<Graphics::Shader>::Instance().GetPtr(m_fontPS);
		pVS->Bind(pDC);
		pPS->Bind(pDC);

		Graphics::Graphics_Impl::Instance().BindUIFrameConstants();

		pDC->IASetInputLayout(m_pFontInputLayout.Get());
		pDC->OMSetDepthStencilState(m_pDepthStencilState.Get(), 0u);
		pDC->OMSetBlendState(m_pBlendState.Get(), NULL, 0xffffffff);
	}

}

//------------------------------------------------ UI/UIApi.cpp ------------------------------------------------

namespace Play3d::UI
{
	FontId GetDebugFont()
	{
		static FontId s_debugFontId;
		if (s_debugFontId.IsInvalid())
		{
			UI::FontDesc desc;
			char charSet[128 + 1];
			u32 j = 0;
			for (u32 i = 0; i < 128; ++i)
			{
				if (isprint(i))
				{
					charSet[j] = (char)i;
					++j;
				}
			}
			charSet[j] = '\0';
			desc.m_fontName = "Consolas";
			desc.m_pointSize = 20;
			desc.m_charSet = charSet;
			desc.m_textureWidth = 512;
			desc.m_textureHeight = 64;
			s_debugFontId = Resources::CreateAsset<UI::Font>(desc);
		}
		return s_debugFontId;
	}

	void DrawPrintf(FontId hFont, const Vector2f& position, ColourValue colour, const char* fmt, ...)
	{
		static constexpr u32 kBufferSize = 1024;
		char strBuffer[kBufferSize];
		va_list args;
		va_start(args, fmt);
		vsprintf_s(strBuffer, kBufferSize, fmt, args);
		va_end(args);

		DrawString(hFont, position, colour, strBuffer);
	}

	void DrawString(FontId hFont, const Vector2f& position, ColourValue colour, std::string_view text)
	{
		Font* pFont = Resources::ResourceManager<Font>::Instance().GetPtr(hFont);
		if (pFont)
		{
			auto* pDC = Graphics::Graphics_Impl::Instance().GetDeviceContext();
			pFont->DrawString(pDC, position, colour, text);
		}
	}

	bool DrawButton(Sprite::SpriteAtlasId hAtlas, Graphics::TextureId texture, u32 index)
	{
		return false;
	}

	void DrawCharacterArray(FontId hFont, const Vector2f& position, const char* pCharArray,
							const ColourValue* pColourArray, u32 lines, u32 columns)
	{
		Font* pFont = Resources::ResourceManager<Font>::Instance().GetPtr(hFont);
		if (pFont)
		{
			auto* pDC = Graphics::Graphics_Impl::Instance().GetDeviceContext();
			pFont->DrawCharacterArray(pDC, position, pCharArray, pColourArray, lines, columns);
		}
	}

}

////////////////// END IMPLEMENTATION SECTION ////////////////////////////
#endif PLAY_IMPLEMENTATION

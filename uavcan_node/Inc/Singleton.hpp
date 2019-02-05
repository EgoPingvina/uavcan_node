#pragma once

/// <summary>
/// Singleton implementation to insert into any other class
/// </summary>
/// <typeparam name="T">A class that will be converted to Singleton</typeparam>
/// <param name="Action">Steps to perform in the default constructor</typeparam>
#define Singleton(T, Action)				\
	protected:								\
		T() { (Action)(); }					\
		T(T const&) = delete;				\
		T& operator=(T const&) = delete;	\
		~T() = default;						\
	public:									\
		static T* Instance()				\
		{									\
			static T instance;				\
			return &instance;				\
		}									\
	private:								\

/**
 * C++ 11 solution to print out variable type
 * Copied from:
 * https://stackoverflow.com/questions/81870/is-it-possible-to-print-a-variables-type-in-standard-c
 * Created by Bogdan on 1/12/2020
 */
#ifndef UTILS_TYPE_PRINTING_H_
#define UTILS_TYPE_PRINTING_H_

#include <type_traits>
#include <typeinfo>
#ifndef _MSC_VER
#include <cxxabi.h>
#endif
#include <memory>
#include <string>
#include <cstdlib>

// tldr use TYPE(variable) to get std::string of the variable's type
#ifndef TYPE
#define TYPE(a) type_name<decltype(a)>()
#endif

template <class T>
std::string type_name()
{
  typedef typename std::remove_reference<T>::type TR;
  std::unique_ptr<char, void (*)(void*)> own(
#ifndef _MSC_VER
      abi::__cxa_demangle(typeid(TR).name(), nullptr, nullptr, nullptr),
#else
      nullptr,
#endif
      std::free);
  std::string r = own != nullptr ? own.get() : typeid(TR).name();
  if (std::is_const<TR>::value)
    r += " const";
  if (std::is_volatile<TR>::value)
    r += " volatile";
  if (std::is_lvalue_reference<T>::value)
    r += "&";
  else if (std::is_rvalue_reference<T>::value)
    r += "&&";
  return r;
}

#endif /* UTILS_TYPE_PRINTING_H_*/

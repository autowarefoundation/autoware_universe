// Copyright (c) 2020 Matthias Fey <matthias.fey@tu-dortmund.de>
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#ifndef AUTOWARE__SCATTER_OPS__REDUCTION_H_
#define AUTOWARE__SCATTER_OPS__REDUCTION_H_

#include <map>
#include <string>

enum ReductionType { SUM, MEAN, MUL, DIV, MIN, MAX };

const std::map<std::string, ReductionType> reduce2REDUCE = {
  {"sum", SUM}, {"mean", MEAN}, {"mul", MUL}, {"div", DIV}, {"min", MIN}, {"max", MAX},
};

const std::map<ReductionType, std::string> REDUCE2reduce = {
  {SUM, "sum"}, {MEAN, "mean"}, {MUL, "mul"}, {DIV, "div"}, {MIN, "min"}, {MAX, "max"},
};

#define AT_DISPATCH_REDUCTION_TYPES(reduce, ...)      \
  [&] {                                               \
    switch (reduce2REDUCE.at(reduce)) {               \
      case SUM: {                                     \
        static constexpr ReductionType REDUCE = SUM;  \
        return __VA_ARGS__();                         \
      }                                               \
      case MEAN: {                                    \
        static constexpr ReductionType REDUCE = MEAN; \
        return __VA_ARGS__();                         \
      }                                               \
      case MUL: {                                     \
        static constexpr ReductionType REDUCE = MUL;  \
        return __VA_ARGS__();                         \
      }                                               \
      case DIV: {                                     \
        static constexpr ReductionType REDUCE = DIV;  \
        return __VA_ARGS__();                         \
      }                                               \
      case MIN: {                                     \
        static constexpr ReductionType REDUCE = MIN;  \
        return __VA_ARGS__();                         \
      }                                               \
      case MAX: {                                     \
        static constexpr ReductionType REDUCE = MAX;  \
        return __VA_ARGS__();                         \
      }                                               \
    }                                                 \
  }()

#endif  // AUTOWARE__SCATTER_OPS__REDUCTION_H_

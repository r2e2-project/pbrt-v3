#ifndef PBRT_UTIL_DEFER_H
#define PBRT_UTIL_DEFER_H

namespace pbrt {

template <typename F>
struct DeferAction {
    DeferAction(F f) : clean_{f} {}
    ~DeferAction() { clean_(); }

  private:
    F clean_;
};

template <typename F>
DeferAction<F> defer(F f) {
    return DeferAction<F>(f);
}

}  // namespace pbrt

#endif

#ifndef __MYWRAPPER_H
#define __MYWRAPPER_H
#ifdef __cplusplus
extern "C" {
#endif


  typedef struct Fuzzy Fuzzy;

  Fuzzy* newFuzzy();

  void deleteFuzzy(Fuzzy* v);


#ifdef __cplusplus
}
#endif
#endif

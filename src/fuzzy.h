// FUZZY BEGIN
// ####################################################################

extern int speed_fuzzy ;
extern int speed_freq ;

#define FIS_TYPE float
#define FIS_RESOLUSION 101
#define FIS_MIN -3.4028235E+38
#define FIS_MAX 3.4028235E+38
typedef FIS_TYPE (*_FIS_MF)(FIS_TYPE, FIS_TYPE *);
typedef FIS_TYPE (*_FIS_ARR_OP)(FIS_TYPE, FIS_TYPE);
typedef FIS_TYPE (*_FIS_ARR)(FIS_TYPE *, int, _FIS_ARR_OP);
#define min(a, b) ((a) < (b) ? (a) : (b))
#define max(a, b) ((a) > (b) ? (a) : (b))

// Number of inputs to the fuzzy inference system
const int fis_gcI = 1;
// Number of outputs to the fuzzy inference system
const int fis_gcO = 1;
// Number of rules to the fuzzy inference system
const int fis_gcR = 5;

FIS_TYPE g_fisInput[fis_gcI];
FIS_TYPE g_fisOutput[fis_gcO];
// FUZZY
//*********************************************************************
FIS_TYPE fis_trapmf(FIS_TYPE x, FIS_TYPE *p)
{
  FIS_TYPE a = p[0], b = p[1], c = p[2], d = p[3];
  FIS_TYPE t1 = ((x <= c) ? 1 : ((d < x) ? 0 : ((c != d) ? ((d - x) / (d - c)) : 0)));
  FIS_TYPE t2 = ((b <= x) ? 1 : ((x < a) ? 0 : ((a != b) ? ((x - a) / (b - a)) : 0)));
  return (FIS_TYPE)min(t1, t2);
}

// Triangular Member Function
FIS_TYPE fis_trimf(FIS_TYPE x, FIS_TYPE *p)
{
  FIS_TYPE a = p[0], b = p[1], c = p[2];
  FIS_TYPE t1 = (x - a) / (b - a);
  FIS_TYPE t2 = (c - x) / (c - b);
  if ((a == b) && (b == c))
    return (FIS_TYPE)(x == a);
  if (a == b)
    return (FIS_TYPE)(t2 * (b <= x) * (x <= c));
  if (b == c)
    return (FIS_TYPE)(t1 * (a <= x) * (x <= b));
  t1 = min(t1, t2);
  return (FIS_TYPE)max(t1, 0);
}

FIS_TYPE fis_prod(FIS_TYPE a, FIS_TYPE b)
{
  return (a * b);
}

FIS_TYPE fis_probor(FIS_TYPE a, FIS_TYPE b)
{
  return (a + b - (a * b));
}

FIS_TYPE fis_sum(FIS_TYPE a, FIS_TYPE b)
{
  return (a + b);
}

FIS_TYPE fis_array_operation(FIS_TYPE *array, int size, _FIS_ARR_OP pfnOp)
{
  int i;
  FIS_TYPE ret = 0;

  if (size == 0)
    return ret;
  if (size == 1)
    return array[0];

  ret = array[0];
  for (i = 1; i < size; i++)
  {
    ret = (*pfnOp)(ret, array[i]);
  }

  return ret;
}

//***********************************************************************
// Data for Fuzzy Inference System
//***********************************************************************
// Pointers to the implementations of member functions
_FIS_MF fis_gMF[] = {
    fis_trapmf, fis_trimf};

// Count of member function for each Input
int fis_gIMFCount[] = {5};

// Count of member function for each Output
int fis_gOMFCount[] = {3};

// Coefficients for the Input Member Functions
FIS_TYPE fis_gMFI0Coeff1[] = {-100, -80, -15, -10};
FIS_TYPE fis_gMFI0Coeff2[] = {-6, 0, 6};
FIS_TYPE fis_gMFI0Coeff3[] = {10, 15, 80, 100};
FIS_TYPE fis_gMFI0Coeff4[] = {-17, -10, -5};
FIS_TYPE fis_gMFI0Coeff5[] = {5, 10, 17};
FIS_TYPE *fis_gMFI0Coeff[] = {fis_gMFI0Coeff1, fis_gMFI0Coeff2, fis_gMFI0Coeff3, fis_gMFI0Coeff4, fis_gMFI0Coeff5};
FIS_TYPE **fis_gMFICoeff[] = {fis_gMFI0Coeff};

// Coefficients for the Output Member Functions
FIS_TYPE fis_gMFO0Coeff1[] = {0, 0};
FIS_TYPE fis_gMFO0Coeff2[] = {0, 1200};
FIS_TYPE fis_gMFO0Coeff3[] = {0, 1400};
FIS_TYPE *fis_gMFO0Coeff[] = {fis_gMFO0Coeff1, fis_gMFO0Coeff2, fis_gMFO0Coeff3};
FIS_TYPE **fis_gMFOCoeff[] = {fis_gMFO0Coeff};

// Input membership function set
int fis_gMFI0[] = {0, 1, 0, 1, 1};
int *fis_gMFI[] = {fis_gMFI0};

// Output membership function set

int *fis_gMFO[] = {};

// Rule Weights
FIS_TYPE fis_gRWeight[] = {1, 1, 1, 1, 1};

// Rule Type
int fis_gRType[] = {1, 1, 1, 1, 1};

// Rule Inputs
int fis_gRI0[] = {1};
int fis_gRI1[] = {4};
int fis_gRI2[] = {2};
int fis_gRI3[] = {5};
int fis_gRI4[] = {3};
int *fis_gRI[] = {fis_gRI0, fis_gRI1, fis_gRI2, fis_gRI3, fis_gRI4};

// Rule Outputs
int fis_gRO0[] = {3};
int fis_gRO1[] = {2};
int fis_gRO2[] = {1};
int fis_gRO3[] = {2};
int fis_gRO4[] = {3};
int *fis_gRO[] = {fis_gRO0, fis_gRO1, fis_gRO2, fis_gRO3, fis_gRO4};

// Input range Min
FIS_TYPE fis_gIMin[] = {-80};

// Input range Max
FIS_TYPE fis_gIMax[] = {80};

// Output range Min
FIS_TYPE fis_gOMin[] = {0};

// Output range Max
FIS_TYPE fis_gOMax[] = {1};

//***********************************************************************
// Data dependent support functions for Fuzzy Inference System
//***********************************************************************
// None for Sugeno
//***********************************************************************
// Fuzzy Inference System
//***********************************************************************
void fis_evaluate()
{
  FIS_TYPE fuzzyInput0[] = {0, 0, 0, 0, 0};
  FIS_TYPE *fuzzyInput[fis_gcI] = {
      fuzzyInput0,
  };
  FIS_TYPE fuzzyOutput0[] = {0, 0, 0};
  FIS_TYPE *fuzzyOutput[fis_gcO] = {
      fuzzyOutput0,
  };
  FIS_TYPE fuzzyRules[fis_gcR] = {0};
  FIS_TYPE fuzzyFires[fis_gcR] = {0};
  FIS_TYPE *fuzzyRuleSet[] = {fuzzyRules, fuzzyFires};
  FIS_TYPE sW = 0;

  // Transforming input to fuzzy Input
  int i, j, r, o;
  for (i = 0; i < fis_gcI; ++i)
  {
    for (j = 0; j < fis_gIMFCount[i]; ++j)
    {
      fuzzyInput[i][j] =
          (fis_gMF[fis_gMFI[i][j]])(g_fisInput[i], fis_gMFICoeff[i][j]);
    }
  }

  int index = 0;
  for (r = 0; r < fis_gcR; ++r)
  {
    if (fis_gRType[r] == 1)
    {
      fuzzyFires[r] = 1;
      for (i = 0; i < fis_gcI; ++i)
      {
        index = fis_gRI[r][i];
        if (index > 0)
          fuzzyFires[r] = fis_prod(fuzzyFires[r], fuzzyInput[i][index - 1]);
        else if (index < 0)
          fuzzyFires[r] = fis_prod(fuzzyFires[r], 1 - fuzzyInput[i][-index - 1]);
        else
          fuzzyFires[r] = fis_prod(fuzzyFires[r], 1);
      }
    }
    else
    {
      fuzzyFires[r] = 0;
      for (i = 0; i < fis_gcI; ++i)
      {
        index = fis_gRI[r][i];
        if (index > 0)
          fuzzyFires[r] = fis_probor(fuzzyFires[r], fuzzyInput[i][index - 1]);
        else if (index < 0)
          fuzzyFires[r] = fis_probor(fuzzyFires[r], 1 - fuzzyInput[i][-index - 1]);
        else
          fuzzyFires[r] = fis_probor(fuzzyFires[r], 0);
      }
    }

    fuzzyFires[r] = fis_gRWeight[r] * fuzzyFires[r];
    sW += fuzzyFires[r];
  }

  if (sW == 0)
  {
    for (o = 0; o < fis_gcO; ++o)
    {
      g_fisOutput[o] = ((fis_gOMax[o] + fis_gOMin[o]) / 2);
    }
  }
  else
  {
    for (o = 0; o < fis_gcO; ++o)
    {
      FIS_TYPE sWI = 0.0;
      for (j = 0; j < fis_gOMFCount[o]; ++j)
      {
        fuzzyOutput[o][j] = fis_gMFOCoeff[o][j][fis_gcI];
        for (i = 0; i < fis_gcI; ++i)
        {
          fuzzyOutput[o][j] += g_fisInput[i] * fis_gMFOCoeff[o][j][i];
        }
      }

      for (r = 0; r < fis_gcR; ++r)
      {
        index = fis_gRO[r][o] - 1;
        sWI += fuzzyFires[r] * fuzzyOutput[o][index];
      }

      g_fisOutput[o] = sWI / sW;
    }
  }
}
// ######################################################################
//  FUZZY END
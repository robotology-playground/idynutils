/* File : pydynutils.i */
%module pydynutils
%include std_string.i
%include std_vector.i

%{
/* Note : always include headers following the inheritance order */

// OpenSoT
#include "idynutils/idynutils.h"
%}

/* Note : always include headers following the inheritance order */
// OpenSoT

%include "idynutils/idynutils.h"

namespace std {
  %template(NamesVector) vector<string>;
};

%feature("autodoc",3);

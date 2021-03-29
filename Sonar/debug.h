//============================================================================
// Copyright   : PerceptIn
//============================================================================
#ifndef _DEBUG_H
#define _DEBUG_H

// #define __DEBUG_STD_OUT__

#ifdef __DEBUG_STD_OUT__
    #define DebugStdOut(x)  x
#else
    #define DebugStdOut(x)
#endif

#endif

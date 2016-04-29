/*============================================================================
==============================================================================
                      
                              initUserGraphics.c
 
==============================================================================
Remarks:

         Functions needed for user graphics
         simulation

============================================================================*/

#include "SL.h"
#include "SL_user.h"
#include "SL_man.h"

// openGL includes
#ifdef powerpc
#include <GLUT/glut.h>
#else
#include "GL/glut.h"
#endif
#include "SL_openGL.h"
#include "SL_userGraphics.h"

// global variables

// local variables

/*****************************************************************************
******************************************************************************
Function Name	: initUserGraphics
Date		: June 1999
   
Remarks:

      allows adding new graphics functions to openGL interface

******************************************************************************
Paramters:  (i/o = input/output)

  none   

*****************************************************************************/
int
initUserGraphics(void)

{

  switchCometDisplay(TRUE,500);

  return TRUE;

}


#include "stdafx.h"
#include <conio.h>
__declspec(align(16)) float zi1[100];
__declspec(align(16)) float zi2[100];
__declspec(align(16)) float zo1[100];
__declspec(align(16)) float zo2[100];

int _tmain(int argc, _TCHAR* argv[])
{
  float a, b, y = 0;
  _cscanf("%f",&a);
  _putch(10);
  _cscanf("%f",&b);
  _putch(10);
  _cprintf("%f %f",a,b);
  _putch(10);
  for(y = 0 ; y < 10.0; y += 0.2f){
    zi1[(int)(y * 5)] = y * y - a;
    zi2[(int)(y * 5)] = y * y + b;
    zo1[(int)(y * 5)] = 1 / (2 * ((y * y * y * y) + (b - a) * y * y - a * b));
  }

  for(int i = 0 ; i < 50; i++){
    zo2[i] = 0.5;
  }

  _asm{
    mov ecx, 0
    loop1:
    movaps xmm2, oword ptr zo2[ecx]
    movaps xmm0, oword ptr zi1[ecx]
    movaps xmm1, oword ptr zi2[ecx]
    mulps xmm0, xmm1
    divps xmm2, xmm0
    movaps oword ptr zo2[ecx], xmm2
    add ecx,16
    cmp ecx,200
    jb loop1
    }
  _cprintf("zi1\tzi2\tzo1\tzo2\t");
  _putch(10);

  for (int i = 0 ; i < 50 ; i++){
    _cprintf("%f\t%f\t%f\t%f\t",zi1[i],zi2[i],zo1[i],zo2[i]);
    _putch(10);
  }
  getche();
  getche();
  return 0;
}
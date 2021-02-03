Stutter+Hold: a timbral tremolo.
================================
SILVER award winning submission to the 2020 AES Student Competition, building an audio plugin in Matlab.

Stutter+Hold is a AU/VST timbral tremolo plug-in built with the MATLAB Audio Toolbox, designed by Julian Vanasse and Max Henry (McGill University, Music Technology Area).

Built on the skeleton of a from-scratch analysis synthesis framework, Stutter+Hold uses a phase vocoding base, combined with an amplitude follower, voiced/unvoiced detection and biologically inspired dynamic compression to create this unique creative tool intended for live or studio use. 

A first plugin for both of us, this code is meat-and-potatoes DSP. Aside from the audioPlugin class itself, we don't make use of any built-in functions. It took on many forms over the months, but we hope this final form, a timbral tremolo, will inspire you in your creative endeavours.

Matlab usage::
  
  % Run live using the built in testbench:
  audioTestBench StutterAndHold
  
  % Build a VST plugin:
  generateAudioPlugin -au StutterAndHold
  
  % Build an AU plugin:
  generateAudioPlugin -vst StutterAndHold

Our source code also can be found at the Matlab central file exchange. A demo video can be seen here:

https://youtu.be/dxMSjbI3etM

Thanks and good luck to everyone.
Max + Julian

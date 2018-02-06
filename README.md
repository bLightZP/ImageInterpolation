# ImageInterpolation
High quality, multi-platform pascal image interpolation (resizing/scaling) using TBitmap (VCL/FMX)

# Supported sampling filters:
Nearest neighbor<br>
Linear<br>
Cosine<br>
Hermite<br>
Quadratic<br>
Gaussian<br>
Spline<br>
Lanczos<br>
Mitchell<br>
CatmullRom<br>

# Introduction
I was having a very hard time finding a high quality image scaling that is free to use, simple to integrate and high in quality.  I finally managed to find usable code in mark mauder's vampyre imaging library.  I modified the code to make it simpler to use with both the VCL and FMX versions of TBitmap.

# Notes:
For simplicity, I only supported 32bit (R/G/B/A) bitmap.<br>
The code is pure pascal, there are no low level CPU-specific optimizations.

# Credits:
Based on the Vampyre Imaging Library by Marek Mauder<br>
https://imaginglib.sourceforge.net

Modified for simplicity of use with TBitmap VCL/FMX classes by Yaron Gur

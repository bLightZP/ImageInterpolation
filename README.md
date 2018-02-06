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
I was having a very hard time finding image scaling (resizing) code that is free to use, simple to integrate and high in quality.  I finally managed to find usable code in mark mauder's vampyre imaging library, but it was wrapped in a lot of code that overly complicated the specific use function I needed.
<br><br>
I ended up modifying the code to make it simpler to use with both the VCL and FMX versions of TBitmap.

# Usage:
<b>StretchResample</b>(const SrcImage: TBitmap; SrcX, SrcY, SrcWidth,SrcHeight: LongInt; var DstImage: TBitmap; DstX, DstY, DstWidth,  DstHeight: LongInt; Filter: TSamplingFilter; WrapEdges: Boolean = False); overload;

# Example of resizing an image 50% :
ResizedImage.Width  := SourceImage.Width div 2;<br>
ResizedImage.Height := SourceImage.Height div 2;<br>
<b>StretchResample</b>(SourceBitmap,0,0, MySourceBitmap.Width, MySourceBitmap.Height,ResizedBitmap,0,0,ResizedBitmap.Width,ResizedBitmap, sfLanczos,False);

# Notes:
For simplicity, I only supported 32bit (R/G/B/A) bitmap.<br>
The code is pure pascal, there are no low level CPU-specific optimizations.

# Credits:
Based on the Vampyre Imaging Library by Marek Mauder<br>
https://imaginglib.sourceforge.net

Modified for simplicity of use with TBitmap VCL/FMX classes by Yaron Gur

{$DEFINE USE_FMX_BITMAP}

{
  Based on the
  Vampyre Imaging Library
  by Marek Mauder
  https://imaginglib.sourceforge.net

  Modified for TBitmap VCL/FMX
  by Yaron Gur
  https://

  The contents of this file are used with permission, subject to the Mozilla
  Public License Version 1.1 (the "License"); you may not use this file except
  in compliance with the License. You may obtain a copy of the License at
  https://www.mozilla.org/MPL/MPL-1.1.html

  Software distributed under the License is distributed on an "AS IS" basis,
  WITHOUT WARRANTY OF ANY KIND, either express or implied. See the License for
  the specific language governing rights and limitations under the License.

  Alternatively, the contents of this file may be used under the terms of the
  GNU Lesser General Public License (the  "LGPL License"), in which case the
  provisions of the LGPL License are applicable instead of those above.
  If you wish to allow use of your version of this file only under the terms
  of the LGPL License and not to allow others to use your version of this file
  under the MPL, indicate your decision by deleting  the provisions above and
  replace  them with the notice and other provisions required by the LGPL
  License.  If you do not delete the provisions above, a recipient may use
  your version of this file under either the MPL or the LGPL License.

  For more information about the LGPL: https://www.gnu.org/copyleft/lesser.html
}

unit resizeimageunit;

interface

uses
  SysUtils, {$IFDEF USE_FMX_BITMAP}FMX.Graphics,FMX.Types{$ELSE}Graphics{$ENDIF};

type
  { Built-in sampling filters.}
  TSamplingFilter = (sfNearest, sfLinear, sfCosine, sfHermite, sfQuadratic, sfGaussian, sfSpline, sfLanczos, sfMitchell, sfCatmullRom);
  { Type of custom sampling function}
  TFilterFunction = function(Value: Single): Single;
  TPointRec       =
  record
    Pos    : LongInt;
    Weight : Single;
  end;
  TCluster        = array of TPointRec;
  TMappingTable   = array of TCluster;

  { Color value for 32 bit images.}
  TColor32 = LongWord;
  PColor32 = ^TColor32;

  { Color record for 32 bit images, which allows access to individual color
    channels.}
  TColor32Rec = packed record
  case LongInt of
    0: (Color: TColor32);
    1: (B, G, R, A: Byte);
    2: (Channels: array[0..3] of Byte);
  end;
  PColor32Rec = ^TColor32Rec;
  TColor32RecArray = array[0..MaxInt div SizeOf(TColor32Rec) - 1] of TColor32Rec;
  PColor32RecArray = ^TColor32RecArray;

const
  { Default resampling filter used for bicubic resizing.}
  DefaultCubicFilter = sfCatmullRom;


var
  { Built-in filter functions.}
  SamplingFilterFunctions : array[TSamplingFilter] of TFilterFunction;
  { Default radii of built-in filter functions.}
  SamplingFilterRadii     : array[TSamplingFilter] of Single;



{ Stretches rectangle in source image to rectangle in destination image
  with resampling. One of built-in resampling filters defined by
  Filter is used. Set WrapEdges to True for seamlessly tileable images.
  SrcImage and DstImage must be in the same data format.
  Works for all data formats except special and indexed formats.}
procedure StretchResample(const SrcImage: TBitmap; SrcX, SrcY, SrcWidth,
  SrcHeight: LongInt; var DstImage: TBitmap; DstX, DstY, DstWidth,
  DstHeight: LongInt; Filter: TSamplingFilter; WrapEdges: Boolean = False); overload;

  { Stretches rectangle in source image to rectangle in destination image
  with resampling. You can use custom sampling function and filter radius.
  Set WrapEdges to True for seamlessly tileable images. SrcImage and DstImage
  must be in the same data format.
  Works for all data formats except special and indexed formats.}
procedure StretchResample(const SrcImage: TBitmap; SrcX, SrcY, SrcWidth,
  SrcHeight: LongInt; var DstImage: TBitmap; DstX, DstY, DstWidth,
  DstHeight: LongInt; Filter: TFilterFunction; Radius: Single;
  WrapEdges: Boolean = False); overload;

{ Helper function for resampling.}
function BuildMappingTable(DstLow, DstHigh, SrcLow, SrcHigh, SrcImageWidth: LongInt; Filter: TFilterFunction; Radius: Single; WrapEdges: Boolean): TMappingTable;

{ Helper function for resampling.}
procedure FindExtremes(const Map: TMappingTable; var MinPos, MaxPos: LongInt);

 { Clamps integer value to range <Min, Max>}
function ClampInt(Number: LongInt; Min, Max: LongInt): LongInt; {$IFDEF USE_INLINE}inline;{$ENDIF}




implementation

uses math;

var
  FullEdge: Boolean = True;


{ The following resampling code is modified and extended code from Graphics32
  library by Alex A. Denisov.}
function BuildMappingTable(DstLow, DstHigh, SrcLow, SrcHigh, SrcImageWidth: LongInt; Filter: TFilterFunction; Radius: Single; WrapEdges: Boolean): TMappingTable;
var
  I, J, K, N: LongInt;
  Left, Right, SrcWidth, DstWidth: LongInt;
  Weight, Scale, Center, Count: Single;
begin
  Result := nil;
  K := 0;
  SrcWidth := SrcHigh - SrcLow;
  DstWidth := DstHigh - DstLow;

  // Check some special cases
  if SrcWidth = 1 then
  begin
    SetLength(Result, DstWidth);
    for I := 0 to DstWidth - 1 do
    begin
      SetLength(Result[I], 1);
      Result[I][0].Pos := 0;
      Result[I][0].Weight := 1.0;
    end;
    Exit;
  end
  else
  if (SrcWidth = 0) or (DstWidth = 0) then
    Exit;

  if FullEdge then
    Scale := DstWidth / SrcWidth
  else
    Scale := (DstWidth - 1) / (SrcWidth - 1);

  SetLength(Result, DstWidth);

  // Pre-calculate filter contributions for a row or column
  if Scale = 0.0 then
  begin
    Assert(Length(Result) = 1);
    SetLength(Result[0], 1);
    Result[0][0].Pos := (SrcLow + SrcHigh) div 2;
    Result[0][0].Weight := 1.0;
  end
  else if Scale < 1.0 then
  begin
    // Sub-sampling - scales from bigger to smaller 
    Radius := Radius / Scale;
    for I := 0 to DstWidth - 1 do
    begin
      if FullEdge then
        Center := SrcLow - 0.5 + (I + 0.5) / Scale
      else
        Center := SrcLow + I / Scale;
      Left := Floor(Center - Radius);
      Right := Ceil(Center + Radius);
      Count := -1.0;
      for J := Left to Right do
      begin
        Weight := Filter((Center - J) * Scale) * Scale;
        if Weight <> 0.0 then
        begin
          Count := Count + Weight;
          K := Length(Result[I]);
          SetLength(Result[I], K + 1);
          Result[I][K].Pos := ClampInt(J, SrcLow, SrcHigh - 1);
          Result[I][K].Weight := Weight;
        end;
      end;
      if Length(Result[I]) = 0 then
      begin
        SetLength(Result[I], 1);
        Result[I][0].Pos := Floor(Center);
        Result[I][0].Weight := 1.0;
      end
      else if Count <> 0.0 then
        Result[I][K div 2].Weight := Result[I][K div 2].Weight - Count;
    end;
  end
  else // if Scale > 1.0 then
  begin
    // Super-sampling - scales from smaller to bigger
    Scale := 1.0 / Scale;
    for I := 0 to DstWidth - 1 do
    begin
      if FullEdge then
        Center := SrcLow - 0.5 + (I + 0.5) * Scale
      else
        Center := SrcLow + I * Scale;
      Left := Floor(Center - Radius);
      Right := Ceil(Center + Radius);
      Count := -1.0;
      for J := Left to Right do
      begin
        Weight := Filter(Center - J);
        if Weight <> 0.0 then
        begin
          Count := Count + Weight;
          K := Length(Result[I]);
          SetLength(Result[I], K + 1);

          if WrapEdges then
          begin
            if J < 0 then
              N := SrcImageWidth + J
            else if J >= SrcImageWidth then
              N := J - SrcImageWidth
            else
              N := ClampInt(J, SrcLow, SrcHigh - 1);
          end
          else
            N := ClampInt(J, SrcLow, SrcHigh - 1);

          Result[I][K].Pos := N;
          Result[I][K].Weight := Weight;
        end;
      end;
      if Count <> 0.0 then
        Result[I][K div 2].Weight := Result[I][K div 2].Weight - Count;
    end;
  end;
end;


procedure FindExtremes(const Map: TMappingTable; var MinPos, MaxPos: LongInt);
var
  I, J: LongInt;
begin
  if Length(Map) > 0 then
  begin
    MinPos := Map[0][0].Pos;
    MaxPos := MinPos;
    for I := 0 to Length(Map) - 1 do
      for J := 0 to Length(Map[I]) - 1 do
      begin
        if MinPos > Map[I][J].Pos then
          MinPos := Map[I][J].Pos;
        if MaxPos < Map[I][J].Pos then
          MaxPos := Map[I][J].Pos;
      end;
  end;
end;


function ClampInt(Number: LongInt; Min, Max: LongInt): LongInt;
begin
  Result := Number;
  if Result < Min then
    Result := Min
  else if Result > Max then
    Result := Max;
end;


{ Filter function for nearest filtering. Also known as box filter.}
function FilterNearest(Value: Single): Single;
begin
  if (Value > -0.5) and (Value <= 0.5) then
    Result := 1
  else
    Result := 0;
end;


{ Filter function for linear filtering. Also known as triangle or Bartlett filter.}
function FilterLinear(Value: Single): Single;
begin
  if Value < 0.0 then
    Value := -Value;
  if Value < 1.0 then
    Result := 1.0 - Value
  else
    Result := 0.0;
end;


{ Cosine filter.}
function FilterCosine(Value: Single): Single;
begin
  Result := 0;
  if Abs(Value) < 1 then
    Result := (Cos(Value * Pi) + 1) / 2;
end;


{ f(t) = 2|t|^3 - 3|t|^2 + 1, -1 <= t <= 1 }
function FilterHermite(Value: Single): Single;
begin
  if Value < 0.0 then
    Value := -Value;
  if Value < 1 then
    Result := (2 * Value - 3) * Sqr(Value) + 1
  else
    Result := 0;
end;


{ Quadratic filter. Also known as Bell.}
function FilterQuadratic(Value: Single): Single;
begin
  if Value < 0.0 then
    Value := -Value;
  if Value < 0.5 then
    Result := 0.75 - Sqr(Value)
  else
  if Value < 1.5 then
  begin
    Value := Value - 1.5;
    Result := 0.5 * Sqr(Value);
  end
  else
    Result := 0.0;
end;


{ Gaussian filter.}
function FilterGaussian(Value: Single): Single;
begin
  Result := Exp(-2.0 * Sqr(Value)) * Sqrt(2.0 / Pi);
end;


{ 4th order (cubic) b-spline filter.}
function FilterSpline(Value: Single): Single;
var
  Temp: Single;
begin
  if Value < 0.0 then
    Value := -Value;
  if Value < 1.0 then
  begin
    Temp := Sqr(Value);
    Result := 0.5 * Temp * Value - Temp + 2.0 / 3.0;
  end
  else
  if Value < 2.0 then
  begin
    Value := 2.0 - Value;
    Result := Sqr(Value) * Value / 6.0;
  end
  else
    Result := 0.0;
end;


{ Lanczos-windowed sinc filter.}
function FilterLanczos(Value: Single): Single;

  function SinC(Value: Single): Single;
  begin
    if Value <> 0.0 then
    begin
      Value := Value * Pi;
      Result := Sin(Value) / Value;
    end
    else
      Result := 1.0;
  end;

begin
  if Value < 0.0 then
    Value := -Value;
  if Value < 3.0 then
    Result := SinC(Value) * SinC(Value / 3.0)
  else
    Result := 0.0;
end;


{ Micthell cubic filter.}
function FilterMitchell(Value: Single): Single;
const
  B = 1.0 / 3.0;
  C = 1.0 / 3.0;
var
  Temp: Single;
begin
  if Value < 0.0 then
    Value := -Value;
  Temp := Sqr(Value);
  if Value < 1.0 then
  begin
    Value := (((12.0 - 9.0 * B - 6.0 * C) * (Value * Temp)) +
      ((-18.0 + 12.0 * B + 6.0 * C) * Temp) +
      (6.0 - 2.0 * B));
    Result := Value / 6.0;
  end
  else
  if Value < 2.0 then
  begin
    Value := (((-B - 6.0 * C) * (Value * Temp)) +
      ((6.0 * B + 30.0 * C) * Temp) +
      ((-12.0 * B - 48.0 * C) * Value) +
      (8.0 * B + 24.0 * C));
    Result := Value / 6.0;
  end
  else
    Result := 0.0;
end;


{ CatmullRom spline filter.}
function FilterCatmullRom(Value: Single): Single;
begin
  if Value < 0.0 then
    Value := -Value;
  if Value < 1.0 then
    Result := 0.5 * (2.0 + Sqr(Value) * (-5.0 + 3.0 * Value))
  else
  if Value < 2.0 then
    Result := 0.5 * (4.0 + Value * (-8.0 + Value * (5.0 - Value)))
  else
    Result := 0.0;
end;


procedure CopyPixel(Src, Dest: Pointer; BytesPerPixel: LongInt);
begin
  case BytesPerPixel of
    1: PByte(Dest)^ := PByte(Src)^;
    2: PWord(Dest)^ := PWord(Src)^;
    //3: PColor24Rec(Dest)^ := PColor24Rec(Src)^;
    4: PLongWord(Dest)^ := PLongWord(Src)^;
    //6: PColor48Rec(Dest)^ := PColor48Rec(Src)^;
    8: PInt64(Dest)^ := PInt64(Src)^;
    //16: PColorFPRec(Dest)^ := PColorFPRec(Src)^;
  end;
end;


procedure StretchResample(const SrcImage: TBitmap; SrcX, SrcY, SrcWidth, SrcHeight: LongInt; var DstImage: TBitmap; DstX, DstY, DstWidth, DstHeight: LongInt;
                          Filter: TSamplingFilter; WrapEdges: Boolean = False);
begin
  // Calls the other function with filter function and radius defined by Filter
  StretchResample(SrcImage, SrcX, SrcY, SrcWidth, SrcHeight, DstImage, DstX, DstY,
    DstWidth, DstHeight, SamplingFilterFunctions[Filter], SamplingFilterRadii[Filter],
    WrapEdges);
end;


procedure StretchResample(const SrcImage: TBitmap; SrcX, SrcY, SrcWidth, SrcHeight: LongInt; var DstImage: TBitmap; DstX, DstY, DstWidth, DstHeight: LongInt;
                          Filter: TFilterFunction; Radius: Single; WrapEdges: Boolean);
const
  Channel8BitMax: Single = 255.0;
type
  TBufferItem = record
    A, R, G, B: Integer;
  end;
var
  MapX, MapY                                  : TMappingTable;
  I, J, X, Y                                  : LongInt;
  XMinimum, XMaximum                          : LongInt;
  LineBufferInt                               : array of TBufferItem;
  ClusterX, ClusterY                          : TCluster;
  Weight, AccumA, AccumR, AccumG, AccumB      : Single;
  IWeight, IAccumA, IAccumR, IAccumG, IAccumB : Integer;
  SrcColor                                    : TColor32Rec;
  //BytesPerChannel                             : LongInt;
  ChannelValueMax, InvChannelValueMax         : Single;
  Speed1                                      : Integer;

  {$IFDEF USE_FMX_BITMAP}
  bitmapData                                  : FMX.Graphics.TBitmapData;
  tmpScanLine                                 : Pointer;
  {$ENDIF}
  srcScanLines                                : Array[0..4095] of PByteArray;
  dstScanLines                                : Array[0..4095] of PByteArray;

begin
  If (SrcImage.Width < 2) or (SrcImage.Height < 2) or (DstImage.Width < 2) or (DstImage.Height < 2) then Exit;

  //BytesPerChannel := 1; // 8bit/channel


  // Create horizontal and vertical mapping tables
  MapX := BuildMappingTable(DstX, DstX + DstWidth , SrcX, SrcX + SrcWidth , SrcImage.Width , Filter, Radius, WrapEdges);
  MapY := BuildMappingTable(DstY, DstY + DstHeight, SrcY, SrcY + SrcHeight, SrcImage.Height, Filter, Radius, WrapEdges);

  if (MapX = nil) or (MapY = nil) then
    Exit;

  ClusterX := nil;
  ClusterY := nil;

  {$IFNDEF USE_FMX_BITMAP}
  // Get scanline pointers
  If SrcImage.PixelFormat <> pf32bit then SrcImage.PixelFormat := pf32bit;
  If DstImage.PixelFormat <> pf32bit then DstImage.PixelFormat := pf32bit;
  For I := 0 to SrcImage.Height-1 do srcScanLines[I] := SrcImage.ScanLine[I];
  For I := 0 to DstImage.Height-1 do dstScanLines[I] := DstImage.ScanLine[I];
  {$ELSE}
  // Convert image to TAlphaColor format
  If SrcImage.Map(TMapAccess.Read, bitmapData) then
  try
    For Y := 0 to SrcImage.Height-1 do
    Begin
      tmpScanLine := bitmapData.GetScanline(Y);
      GetMem(srcScanLines[Y],SrcImage.Width*4);
      ScanlineToAlphaColor(tmpScanLine,@srcScanLines[Y][0],SrcImage.Width,SrcImage.PixelFormat);
    End;
  finally
    SrcImage.Unmap(bitmapData);
  end;
  For Y := 0 to DstImage.Height-1 do GetMem(dstScanLines[Y],DstImage.Width*4);
  {$ENDIF}

  try
    // Find min and max X coords of pixels that will contribute to target image
    FindExtremes(MapX, XMinimum, XMaximum);

    SetLength(LineBufferInt, XMaximum - XMinimum + 1);

    // Following code is optimized for images with 8 bit channels
    for J := 0 to DstHeight - 1 do
    begin
      ClusterY := MapY[J];
      for X := XMinimum to XMaximum do
      begin
        IAccumA := 0;
        IAccumR := 0;
        IAccumG := 0;
        IAccumB := 0;
        for Y := 0 to Length(ClusterY) - 1 do
        begin
          IWeight := Round(256 * ClusterY[Y].Weight);
          Speed1  := X*4;

          IAccumB := IAccumB + srcScanLines[ClusterY[Y].Pos][Speed1  ] * IWeight;
          IAccumG := IAccumG + srcScanLines[ClusterY[Y].Pos][Speed1+1] * IWeight;
          IAccumR := IAccumR + srcScanLines[ClusterY[Y].Pos][Speed1+2] * IWeight;
          IAccumA := IAccumA + srcScanLines[ClusterY[Y].Pos][Speed1+3] * IWeight;
        end;
        with LineBufferInt[X - XMinimum] do
        begin
          A := IAccumA;
          R := IAccumR;
          G := IAccumG;
          B := IAccumB;
        end;
      end;

      //DstLine := @PByteArray(DstImage.Bits)[((J + DstY) * DstImage.Width + DstX)* Info.BytesPerPixel];
      for I := 0 to DstWidth - 1 do
      begin
        ClusterX := MapX[I];
        IAccumA := 0;
        IAccumR := 0;
        IAccumG := 0;
        IAccumB := 0;
        for X := 0 to Length(ClusterX) - 1 do
        begin
          IWeight := Round(256 * ClusterX[X].Weight);
          with LineBufferInt[ClusterX[X].Pos - XMinimum] do
          begin
            IAccumB := IAccumB + B * IWeight;
            IAccumG := IAccumG + G * IWeight;
            IAccumR := IAccumR + R * IWeight;
            IAccumA := IAccumA + A * IWeight;
          end;
        end;

        SrcColor.B := ClampInt(IAccumB, 0, $00FF0000) shr 16;
        SrcColor.G := ClampInt(IAccumG, 0, $00FF0000) shr 16;
        SrcColor.R := ClampInt(IAccumR, 0, $00FF0000) shr 16;
        SrcColor.A := ClampInt(IAccumA, 0, $00FF0000) shr 16;

        PLongWord(@dstScanLines[J]^[(I+DstX)*4])^ := PLongWord(@SrcColor)^;
      end;
    end;
  finally

    {$IFDEF USE_FMX_BITMAP}
    // Convert TAlphaColor format back to Image format
    If DstImage.Map(TMapAccess.Write, bitmapData) then
    try
      For Y := 0 to DstImage.Height-1 do
      Begin
        tmpScanLine := bitmapData.GetScanline(Y);
        AlphaColorToScanLine(@dstScanLines[Y][0],tmpScanLine,DstImage.Width,DstImage.PixelFormat);
      End;
    finally
      DstImage.Unmap(bitmapData);
    end;

    for Y := 0 to SrcImage.Height-1 do FreeMem(srcScanLines[Y]);
    for Y := 0 to DstImage.Height-1 do FreeMem(dstScanLines[Y]);
    {$ENDIF}

    SrcImage.SaveToFile('d:\x\@test_src.bmp');
    DstImage.SaveToFile('d:\x\@test_dst.bmp');

    MapX := nil;
    MapY := nil;
  end;
end;


initialization
  // Initialize default sampling filter function pointers and radii
  SamplingFilterFunctions[sfNearest]    := FilterNearest;
  SamplingFilterFunctions[sfLinear]     := FilterLinear;
  SamplingFilterFunctions[sfCosine]     := FilterCosine;
  SamplingFilterFunctions[sfHermite]    := FilterHermite;
  SamplingFilterFunctions[sfQuadratic]  := FilterQuadratic;
  SamplingFilterFunctions[sfGaussian]   := FilterGaussian;
  SamplingFilterFunctions[sfSpline]     := FilterSpline;
  SamplingFilterFunctions[sfLanczos]    := FilterLanczos;
  SamplingFilterFunctions[sfMitchell]   := FilterMitchell;
  SamplingFilterFunctions[sfCatmullRom] := FilterCatmullRom;
  SamplingFilterRadii[sfNearest]    := 1.0;
  SamplingFilterRadii[sfLinear]     := 1.0;
  SamplingFilterRadii[sfCosine]     := 1.0;
  SamplingFilterRadii[sfHermite]    := 1.0;
  SamplingFilterRadii[sfQuadratic]  := 1.5;
  SamplingFilterRadii[sfGaussian]   := 1.25;
  SamplingFilterRadii[sfSpline]     := 2.0;
  SamplingFilterRadii[sfLanczos]    := 3.0;
  SamplingFilterRadii[sfMitchell]   := 2.0;
  SamplingFilterRadii[sfCatmullRom] := 2.0;



end.
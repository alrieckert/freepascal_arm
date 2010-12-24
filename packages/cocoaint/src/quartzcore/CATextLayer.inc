{ Parsed from Quartzcore.framework CATextLayer.h }
{ Version: 2.1.2 - Wed Dec 8 10:06:07 CET 2010 }


{$ifdef TYPES}
{$ifndef CATEXTLAYER_PAS_T}
{$define CATEXTLAYER_PAS_T}

{$endif}
{$endif}

{$ifdef RECORDS}
{$ifndef CATEXTLAYER_PAS_R}
{$define CATEXTLAYER_PAS_R}

{$endif}
{$endif}

{$ifdef FUNCTIONS}
{$ifndef CATEXTLAYER_PAS_F}
{$define CATEXTLAYER_PAS_F}

{$endif}
{$endif}

{$ifdef EXTERNAL_SYMBOLS}
{$ifndef CATEXTLAYER_PAS_S}
{$define CATEXTLAYER_PAS_S}

{ External string constants }
var
  kCATruncationNone: NSString; cvar; external;
  kCATruncationStart: NSString; cvar; external;
  kCATruncationEnd: NSString; cvar; external;
  kCATruncationMiddle: NSString; cvar; external;
  kCAAlignmentNatural: NSString; cvar; external;
  kCAAlignmentLeft: NSString; cvar; external;
  kCAAlignmentRight: NSString; cvar; external;
  kCAAlignmentCenter: NSString; cvar; external;
  kCAAlignmentJustified: NSString; cvar; external;

{$endif}
{$endif}

{$ifdef FORWARD}
  CATextLayer = objcclass;
  CATextLayerPointer = ^CATextLayer;
  CATextLayerPtr = CATextLayerPointer;

{$endif}

{$ifdef CLASSES}
{$ifndef CATEXTLAYER_PAS_C}
{$define CATEXTLAYER_PAS_C}

{ CATextLayer }
  CATextLayer = objcclass(CALayer)
  private
    __state: Pointer;
    
  public
    class function alloc: CATextLayer; message 'alloc';

    procedure setString (newValue: id); message 'setString:';
    function string_: id; message 'string_';
    procedure setFont (newValue: CFTypeRef); message 'setFont:';
    function font: CFTypeRef; message 'font';
    procedure setFontSize (newValue: CGFloat); message 'setFontSize:';
    function fontSize: CGFloat; message 'fontSize';
    procedure setForegroundColor (newValue: CGColorRef); message 'setForegroundColor:';
    function foregroundColor: CGColorRef; message 'foregroundColor';
    procedure setWrapped (newValue: Boolean); message 'setWrapped:';
    function isWrapped: Boolean; message 'isWrapped';
    procedure setTruncationMode (newValue: NSString); message 'setTruncationMode:';
    function truncationMode: NSString; message 'truncationMode';
    procedure setAlignmentMode (newValue: NSString); message 'setAlignmentMode:';
    function alignmentMode: NSString; message 'alignmentMode';
  end; external;

{$endif}
{$endif}
{$ifndef ALLPACKAGES}
{$mode objfpc}{$H+}
program fpmake;

uses fpmkunit;

Var
  T : TTarget;
  P : TPackage;
begin
  With Installer do
    begin
{$endif ALLPACKAGES}

    P:=AddPackage('gmp');
{$ifdef ALLPACKAGES}
    P.Directory:='gmp';
{$endif ALLPACKAGES}
    P.Version:='2.7.1';

    P.Author := 'FreePascal development team';
    P.License := 'LGPL with modification, ';
    P.HomepageURL := 'www.freepascal.org';
    P.Email := '';
    P.Description := 'GMP';
    P.NeedLibC:= false;
    P.OSes := [freebsd,darwin,iphonesim,linux,win32,aix];

    P.SourcePath.Add('src');

    T:=P.Targets.AddUnit('gmp.pas');

    P.Sources.AddExampleFiles('examples/*',false,'.');

{$ifndef ALLPACKAGES}
    Run;
    end;
end.
{$endif ALLPACKAGES}

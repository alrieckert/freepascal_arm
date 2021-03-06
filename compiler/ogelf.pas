{
    Copyright (c) 1998-2006 by Peter Vreman

    Contains the binary elf writer

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.

 ****************************************************************************
}
unit ogelf;

{$i fpcdefs.inc}

interface

    uses
       { common }
       cclasses,globtype,
       { target }
       systems,
       { assembler }
       cpuinfo,cpubase,aasmbase,aasmtai,aasmdata,assemble,
       { output }
       ogbase,
       owbase;

    type
       TElfObjSection = class(TObjSection)
       public
          shstridx,
          shtype,
          shflags,
          shlink,
          shinfo,
          shentsize : longint;
          constructor create(AList:TFPHashObjectList;const Aname:string;Aalign:shortint;Aoptions:TObjSectionOptions);override;
          constructor create_ext(aobjdata:TObjData;const Aname:string;Ashtype,Ashflags:longint;Aalign:shortint;Aentsize:longint);
          constructor create_reloc(aobjdata:TObjData;const Aname:string;allocflag:boolean);
          procedure writeReloc_internal(aTarget:TObjSection;offset:aword;len:byte;reltype:TObjRelocationType);override;
       end;

       TElfSymtabKind = (esk_obj,esk_exe,esk_dyn);

       TElfSymtab = class(TElfObjSection)
       public
         kind: TElfSymtabKind;
         fstrsec: TObjSection;
         symidx: longint;
         constructor create(aObjData:TObjData;aKind:TElfSymtabKind);reintroduce;
         procedure writeSymbol(objsym:TObjSymbol;nameidx:longword=0);
         procedure writeInternalSymbol(avalue:aword;astridx:longword;ainfo:byte;ashndx:word);
       end;

       TElfObjData = class(TObjData)
       public
         constructor create(const n:string);override;
         function  sectionname(atype:TAsmSectiontype;const aname:string;aorder:TAsmSectionOrder):string;override;
         procedure CreateDebugSections;override;
         procedure writereloc(data:aint;len:aword;p:TObjSymbol;reltype:TObjRelocationType);override;
       end;

       TElfObjectOutput = class(tObjOutput)
       private
         symtabsect: TElfSymtab;
         shstrtabsect: TElfObjSection;
         procedure createrelocsection(s:TElfObjSection;data:TObjData);
         procedure createshstrtab(data:TObjData);
         procedure createsymtab(data: TObjData);
         procedure writesectionheader(s:TElfObjSection);
         procedure section_write_symbol(p:TObject;arg:pointer);
         procedure section_write_sh_string(p:TObject;arg:pointer);
         procedure section_count_sections(p:TObject;arg:pointer);
         procedure section_create_relocsec(p:TObject;arg:pointer);
         procedure section_write_sechdr(p:TObject;arg:pointer);
       protected
         function encodereloc(objrel:TObjRelocation):byte;virtual;abstract;
         function writedata(data:TObjData):boolean;override;
       public
         constructor Create(AWriter:TObjectWriter);override;
       end;



implementation

      uses
        SysUtils,
        verbose,
        cutils,globals,fmodule;

    const
      symbolresize = 200*18;

    const
      { ELFHeader.file_class }
      ELFCLASSNONE  = 0;
      ELFCLASS32    = 1;
      ELFCLASS64    = 2;

      { ELFHeader.e_type }
      ET_NONE       = 0;
      ET_REL        = 1;
      ET_EXEC       = 2;
      ET_DYN        = 3;
      ET_CORE       = 4;

      { ELFHeader.e_machine }
      EM_SPARC      = 2;
      EM_386        = 3;
      EM_M68K       = 4;
      EM_PPC        = 20;
      EM_ARM        = 40;
      EM_X86_64     = 62;

{$ifdef sparc}
      ELFMACHINE = EM_SPARC;
{$endif sparc}
{$ifdef i386}
      ELFMACHINE = EM_386;
{$endif i386}
{$ifdef m68k}
      ELFMACHINE = EM_M68K;
{$endif m68k}
{$ifdef powerpc}
      ELFMACHINE = EM_PPC;
{$endif powerpc}
{$ifdef arm}
      ELFMACHINE = EM_ARM;
{$endif arm}
{$ifdef x86_64}
      ELFMACHINE = EM_X86_64;
{$endif x86_64}

      SHN_UNDEF     = 0;
      SHN_ABS       = $fff1;
      SHN_COMMON    = $fff2;

      SHT_NULL     = 0;
      SHT_PROGBITS = 1;
      SHT_SYMTAB   = 2;
      SHT_STRTAB   = 3;
      SHT_RELA     = 4;
      SHT_HASH     = 5;
      SHT_DYNAMIC  = 6;
      SHT_NOTE     = 7;
      SHT_NOBITS   = 8;
      SHT_REL      = 9;
      SHT_SHLIB    = 10;
      SHT_DYNSYM   = 11;

      SHF_WRITE     = 1;
      SHF_ALLOC     = 2;
      SHF_EXECINSTR = 4;

      STB_LOCAL   = 0;
      STB_GLOBAL  = 1;
      STB_WEAK    = 2;

      STT_NOTYPE  = 0;
      STT_OBJECT  = 1;
      STT_FUNC    = 2;
      STT_SECTION = 3;
      STT_FILE    = 4;
      STT_COMMON  = 5;
      STT_TLS     = 6;
      STT_GNU_IFUNC = 10;

      { program header types }
      PT_NULL     = 0;
      PT_LOAD     = 1;
      PT_DYNAMIC  = 2;
      PT_INTERP   = 3;
      PT_NOTE     = 4;
      PT_SHLIB    = 5;
      PT_PHDR     = 6;
      PT_LOOS     = $60000000;
      PT_HIOS     = $6FFFFFFF;
      PT_LOPROC   = $70000000;
      PT_HIPROC   = $7FFFFFFF;
      PT_GNU_EH_FRAME = PT_LOOS + $474e550;   { Frame unwind information }
      PT_GNU_STACK = PT_LOOS + $474e551;      { Stack flags }
      PT_GNU_RELRO = PT_LOOS + $474e552;      { Read-only after relocation }

      { program header flags }
      PF_X = 1;
      PF_W = 2;
      PF_R = 4;
      PF_MASKOS   = $0FF00000;   { OS-specific reserved bits }
      PF_MASKPROC = $F0000000;   { Processor-specific reserved bits }

      { .dynamic tags  }
      DT_NULL     = 0;
      DT_NEEDED   = 1;
      DT_PLTRELSZ = 2;
      DT_PLTGOT   = 3;
      DT_HASH     = 4;
      DT_STRTAB   = 5;
      DT_SYMTAB	  = 6;
      DT_RELA     = 7;
      DT_RELASZ   = 8;
      DT_RELAENT  = 9;
      DT_STRSZ    = 10;
      DT_SYMENT   = 11;
      DT_INIT     = 12;
      DT_FINI     = 13;
      DT_SONAME   = 14;
      DT_RPATH    = 15;
      DT_SYMBOLIC = 16;
      DT_REL      = 17;
      DT_RELSZ    = 18;
      DT_RELENT   = 19;
      DT_PLTREL   = 20;
      DT_DEBUG    = 21;
      DT_TEXTREL  = 22;
      DT_JMPREL   = 23;
      DT_BIND_NOW = 24;
      DT_INIT_ARRAY = 25;
      DT_FINI_ARRAY = 26;
      DT_INIT_ARRAYSZ = 27;
      DT_FINI_ARRAYSZ = 28;
      DT_RUNPATH  = 29;
      DT_FLAGS    = 30;
      DT_ENCODING = 32;
      DT_PREINIT_ARRAY   = 32;
      DT_PREINIT_ARRAYSZ = 33;
      DT_NUM      = 34;
      DT_LOOS     = $6000000D;
      DT_HIOS     = $6ffff000;
      DT_LOPROC   = $70000000;
      DT_HIPROC   = $7fffffff;

      DT_RELACOUNT = $6ffffff9;
      DT_RELCOUNT  = $6ffffffa;
      DT_FLAGS_1   = $6ffffffb;
      DT_VERDEF    = $6ffffffc;
      DT_VERDEFNUM = $6ffffffd;
      DT_VERNEED   = $6ffffffe;
      DT_VERNEEDNUM = $6fffffff;

      type
      { Structures which are written directly to the output file }
        TElf32header=packed record
          magic             : array[0..3] of byte;
          file_class        : byte;
          data_encoding     : byte;
          file_version      : byte;
          padding           : array[$07..$0f] of byte;
          e_type            : word;
          e_machine         : word;
          e_version         : longword;
          e_entry           : longword;         { entrypoint }
          e_phoff           : longword;         { program header offset }
          e_shoff           : longword;         { sections header offset }
          e_flags           : longword;
          e_ehsize          : word;             { elf header size in bytes }
          e_phentsize       : word;             { size of an entry in the program header array }
          e_phnum           : word;             { 0..e_phnum-1 of entrys }
          e_shentsize       : word;             { size of an entry in sections header array }
          e_shnum           : word;             { 0..e_shnum-1 of entrys }
          e_shstrndx        : word;             { index of string section header }
        end;
        TElf32sechdr=packed record
          sh_name           : longword;
          sh_type           : longword;
          sh_flags          : longword;
          sh_addr           : longword;
          sh_offset         : longword;
          sh_size           : longword;
          sh_link           : longword;
          sh_info           : longword;
          sh_addralign      : longword;
          sh_entsize        : longword;
        end;
        TElf32proghdr=packed record
          p_type            : longword;
          p_offset          : longword;
          p_vaddr           : longword;
          p_paddr           : longword;
          p_filesz          : longword;
          p_memsz           : longword;
          p_flags           : longword;
          p_align           : longword;
        end;
        TElf32reloc=packed record
          address : longword;
          info    : longword; { bit 0-7: type, 8-31: symbol }
          addend  : longint;
        end;
        TElf32symbol=packed record
          st_name  : longword;
          st_value : longword;
          st_size  : longword;
          st_info  : byte; { bit 0-3: type, 4-7: bind }
          st_other : byte;
          st_shndx : word;
        end;
        TElf32Dyn=packed record
          d_tag: longword;
          case integer of
            0: (d_val: longword);
            1: (d_ptr: longword);
        end;


        telf64header=packed record
          magic             : array[0..3] of byte;
          file_class        : byte;
          data_encoding     : byte;
          file_version      : byte;
          padding           : array[$07..$0f] of byte;
          e_type            : word;
          e_machine         : word;
          e_version         : longword;
          e_entry           : qword;            { entrypoint }
          e_phoff           : qword;            { program header offset }
          e_shoff           : qword;            { sections header offset }
          e_flags           : longword;
          e_ehsize          : word;             { elf header size in bytes }
          e_phentsize       : word;             { size of an entry in the program header array }
          e_phnum           : word;             { 0..e_phnum-1 of entrys }
          e_shentsize       : word;             { size of an entry in sections header array }
          e_shnum           : word;             { 0..e_shnum-1 of entrys }
          e_shstrndx        : word;             { index of string section header }
        end;
        telf64sechdr=packed record
          sh_name           : longword;
          sh_type           : longword;
          sh_flags          : qword;
          sh_addr           : qword;
          sh_offset         : qword;
          sh_size           : qword;
          sh_link           : longword;
          sh_info           : longword;
          sh_addralign      : qword;
          sh_entsize        : qword;
        end;
        telf64proghdr=packed record
          p_type            : longword;
          p_flags           : longword;
          p_offset          : qword;
          p_vaddr           : qword;
          p_paddr           : qword;
          p_filesz          : qword;
          p_memsz           : qword;
          p_align           : qword;
        end;
        telf64reloc=packed record
          address : qword;
          info    : qword; { bit 0-31: type, 32-63: symbol }
          addend  : int64; { signed! }
        end;
        telf64symbol=packed record
          st_name  : longword;
          st_info  : byte; { bit 0-3: type, 4-7: bind }
          st_other : byte;
          st_shndx : word;
          st_value : qword;
          st_size  : qword;
        end;
        TElf64Dyn=packed record
          d_tag: qword;
          case integer of
            0: (d_val: qword);
            1: (d_ptr: qword);
        end;

        TElfVerdef=record        { same for 32 and 64 bits }
          vd_version: word;      { =1 }
          vd_flags:   word;
          vd_ndx:     word;
          vd_cnt:     word;      { number of verdaux records }
          vd_hash:    longword;  { ELF hash of version name }
          vd_aux:     longword;  { offset to verdaux records }
          vd_next:    longword;  { offset to next verdef record }
        end;

        TElfVerdaux=record
          vda_name: longword;
          vda_next: longword;
        end;

        TElfVerneed=record
          vn_version: word;      { =VER_NEED_CURRENT }
          vn_cnt:     word;
          vn_file:    longword;
          vn_aux:     longword;
          vn_next:    longword;
        end;

        TElfVernaux=record
          vna_hash:  longword;
          vna_flags: word;
          vna_other: word;
          vna_name:  longword;
          vna_next:  longword;
        end;

{$ifdef cpu64bitaddr}
      const
        ELFCLASS = ELFCLASS64;
      type
        telfheader = telf64header;
        telfreloc = telf64reloc;
        telfsymbol = telf64symbol;
        telfsechdr = telf64sechdr;
        telfproghdr = telf64proghdr;
        telfdyn = telf64dyn;

      function ELF_R_INFO(sym:longword;typ:byte):qword;inline;
        begin
          result:=(qword(sym) shl 32) or typ;
        end;

{$else cpu64bitaddr}
      const
        ELFCLASS = ELFCLASS32;
      type
        telfheader = telf32header;
        telfreloc = telf32reloc;
        telfsymbol = telf32symbol;
        telfsechdr = telf32sechdr;
        telfproghdr = telf32proghdr;
        telfdyn = telf32dyn;

      function ELF_R_INFO(sym:longword;typ:byte):longword;inline;
        begin
          result:=(sym shl 8) or typ;
        end;
{$endif cpu64bitaddr}

{$ifdef x86_64}
      const
        relocs_use_addend:Boolean=True;
{$else x86_64}
      const
        relocs_use_addend:Boolean=False;
{$endif x86_64}

      procedure MayBeSwapHeader(var h : telf32header);
        begin
          if source_info.endian<>target_info.endian then
            with h do
              begin
                e_type:=swapendian(e_type);
                e_machine:=swapendian(e_machine);
                e_version:=swapendian(e_version);
                e_entry:=swapendian(e_entry);
                e_phoff:=swapendian(e_phoff);
                e_shoff:=swapendian(e_shoff);
                e_flags:=swapendian(e_flags);
                e_ehsize:=swapendian(e_ehsize);
                e_phentsize:=swapendian(e_phentsize);
                e_phnum:=swapendian(e_phnum);
                e_shentsize:=swapendian(e_shentsize);
                e_shnum:=swapendian(e_shnum);
                e_shstrndx:=swapendian(e_shstrndx);
              end;
        end;


      procedure MayBeSwapHeader(var h : telf64header);
        begin
          if source_info.endian<>target_info.endian then
            with h do
              begin
                e_type:=swapendian(e_type);
                e_machine:=swapendian(e_machine);
                e_version:=swapendian(e_version);
                e_entry:=swapendian(e_entry);
                e_phoff:=swapendian(e_phoff);
                e_shoff:=swapendian(e_shoff);
                e_flags:=swapendian(e_flags);
                e_ehsize:=swapendian(e_ehsize);
                e_phentsize:=swapendian(e_phentsize);
                e_phnum:=swapendian(e_phnum);
                e_shentsize:=swapendian(e_shentsize);
                e_shnum:=swapendian(e_shnum);
                e_shstrndx:=swapendian(e_shstrndx);
              end;
        end;


      procedure MayBeSwapHeader(var h : telf32proghdr);
        begin
          if source_info.endian<>target_info.endian then
            with h do
              begin
                p_align:=swapendian(p_align);
                p_filesz:=swapendian(p_filesz);
                p_flags:=swapendian(p_flags);
                p_memsz:=swapendian(p_memsz);
                p_offset:=swapendian(p_offset);
                p_paddr:=swapendian(p_paddr);
                p_type:=swapendian(p_type);
                p_vaddr:=swapendian(p_vaddr);
              end;
        end;


      procedure MayBeSwapHeader(var h : telf64proghdr);
        begin
          if source_info.endian<>target_info.endian then
            with h do
              begin
                p_align:=swapendian(p_align);
                p_filesz:=swapendian(p_filesz);
                p_flags:=swapendian(p_flags);
                p_memsz:=swapendian(p_memsz);
                p_offset:=swapendian(p_offset);
                p_paddr:=swapendian(p_paddr);
                p_type:=swapendian(p_type);
                p_vaddr:=swapendian(p_vaddr);
              end;
        end;


      procedure MaybeSwapSecHeader(var h : telf32sechdr);
        begin
          if source_info.endian<>target_info.endian then
            with h do
              begin
                sh_name:=swapendian(sh_name);
                sh_type:=swapendian(sh_type);
                sh_flags:=swapendian(sh_flags);
                sh_addr:=swapendian(sh_addr);
                sh_offset:=swapendian(sh_offset);
                sh_size:=swapendian(sh_size);
                sh_link:=swapendian(sh_link);
                sh_info:=swapendian(sh_info);
                sh_addralign:=swapendian(sh_addralign);
                sh_entsize:=swapendian(sh_entsize);
              end;
        end;


      procedure MaybeSwapSecHeader(var h : telf64sechdr);
        begin
          if source_info.endian<>target_info.endian then
            with h do
              begin
                sh_name:=swapendian(sh_name);
                sh_type:=swapendian(sh_type);
                sh_flags:=swapendian(sh_flags);
                sh_addr:=swapendian(sh_addr);
                sh_offset:=swapendian(sh_offset);
                sh_size:=swapendian(sh_size);
                sh_link:=swapendian(sh_link);
                sh_info:=swapendian(sh_info);
                sh_addralign:=swapendian(sh_addralign);
                sh_entsize:=swapendian(sh_entsize);
              end;
        end;


      procedure MaybeSwapElfSymbol(var h : telf32symbol);
        begin
          if source_info.endian<>target_info.endian then
            with h do
              begin
                st_name:=swapendian(st_name);
                st_value:=swapendian(st_value);
                st_size:=swapendian(st_size);
                st_shndx:=swapendian(st_shndx);
              end;
        end;


      procedure MaybeSwapElfSymbol(var h : telf64symbol);
        begin
          if source_info.endian<>target_info.endian then
            with h do
              begin
                st_name:=swapendian(st_name);
                st_value:=swapendian(st_value);
                st_size:=swapendian(st_size);
                st_shndx:=swapendian(st_shndx);
              end;
        end;


      procedure MaybeSwapElfReloc(var h : telf32reloc);
        begin
          if source_info.endian<>target_info.endian then
            with h do
              begin
                address:=swapendian(address);
                info:=swapendian(info);
                addend:=swapendian(addend);
              end;
        end;


      procedure MaybeSwapElfReloc(var h : telf64reloc);
        begin
          if source_info.endian<>target_info.endian then
            with h do
              begin
                address:=swapendian(address);
                info:=swapendian(info);
                addend:=swapendian(addend);
              end;
        end;


      procedure MaybeSwapElfDyn(var h : telf32dyn);
        begin
          if source_info.endian<>target_info.endian then
            with h do
              begin
                d_tag:=swapendian(d_tag);
                d_val:=swapendian(d_val);
              end;
        end;


      procedure MaybeSwapElfDyn(var h : telf64dyn);
        begin
          if source_info.endian<>target_info.endian then
            with h do
              begin
                d_tag:=swapendian(d_tag);
                d_val:=swapendian(d_val);
              end;
        end;


{****************************************************************************
                                Helpers
****************************************************************************}

    procedure encodesechdrflags(aoptions:TObjSectionOptions;out AshType:longint;out Ashflags:longint);
      begin
        { Section Type }
        AshType:=SHT_PROGBITS;
        if oso_strings in aoptions then
          AshType:=SHT_STRTAB
        else if not(oso_data in aoptions) then
          AshType:=SHT_NOBITS;
        { Section Flags }
        Ashflags:=0;
        if oso_load in aoptions then
          Ashflags:=Ashflags or SHF_ALLOC;
        if oso_executable in aoptions then
          Ashflags:=Ashflags or SHF_EXECINSTR;
        if oso_write in aoptions then
          Ashflags:=Ashflags or SHF_WRITE;
      end;


    procedure decodesechdrflags(AshType:longint;Ashflags:longint;out aoptions:TObjSectionOptions);
      begin
        aoptions:=[];
        { Section Type }
        if AshType<>SHT_NOBITS then
          include(aoptions,oso_data);
        if AshType=SHT_STRTAB then
          include(aoptions,oso_strings);
        { Section Flags }
        if Ashflags and SHF_ALLOC<>0 then
          include(aoptions,oso_load);
        if Ashflags and SHF_WRITE<>0 then
          include(aoptions,oso_write);
        if Ashflags and SHF_EXECINSTR<>0 then
          include(aoptions,oso_executable);
      end;


{****************************************************************************
                               TElfObjSection
****************************************************************************}

    constructor TElfObjSection.create(AList:TFPHashObjectList;const Aname:string;Aalign:shortint;Aoptions:TObjSectionOptions);
      begin
        inherited create(AList,Aname,Aalign,aoptions);
        index:=0;
        shstridx:=0;
        encodesechdrflags(aoptions,shtype,shflags);
        shlink:=0;
        shinfo:=0;
        if name='.stab' then
          shentsize:=sizeof(TObjStabEntry);
      end;


    constructor TElfObjSection.create_ext(aobjdata:TObjData;const Aname:string;Ashtype,Ashflags:longint;Aalign:shortint;Aentsize:longint);
      var
        aoptions : TObjSectionOptions;
      begin
        decodesechdrflags(Ashtype,Ashflags,aoptions);
        inherited create(aobjdata.ObjSectionList,Aname,Aalign,aoptions);
        objdata:=aobjdata;
        index:=0;
        shstridx:=0;
        shtype:=AshType;
        shflags:=AshFlags;
        shentsize:=Aentsize;
      end;


    const
      relsec_prefix:array[boolean] of string[5] = ('.rel','.rela');
      relsec_shtype:array[boolean] of longword = (SHT_REL,SHT_RELA);

    constructor TElfObjSection.create_reloc(aobjdata:TObjData;const Aname:string;allocflag:boolean);
      begin
        create_ext(aobjdata,
          relsec_prefix[relocs_use_addend]+aname,
          relsec_shtype[relocs_use_addend],
          SHF_ALLOC*ord(allocflag),
          sizeof(pint),
          (2+ord(relocs_use_addend))*sizeof(pint));
      end;


    procedure TElfObjSection.writeReloc_internal(aTarget:TObjSection;offset:aword;len:byte;reltype:TObjRelocationType);
      var
        reloc: TObjRelocation;
      begin
        reloc:=TObjRelocation.CreateSection(Size,aTarget,reltype);
        reloc.size:=len;
        ObjRelocations.Add(reloc);
        if reltype=RELOC_RELATIVE then
          dec(offset,len)
        else if reltype<>RELOC_ABSOLUTE then
          InternalError(2012062401);
        if relocs_use_addend then
          begin
            reloc.orgsize:=offset;
            offset:=0;
          end;
        write(offset,len);
      end;


{****************************************************************************
                            TElfObjData
****************************************************************************}

    constructor TElfObjData.create(const n:string);
      begin
        inherited create(n);
        CObjSection:=TElfObjSection;
      end;


    function TElfObjData.sectionname(atype:TAsmSectiontype;const aname:string;aorder:TAsmSectionOrder):string;
      const
        secnames : array[TAsmSectiontype] of string[length('__DATA, __datacoal_nt,coalesced')] = ('','',
{$ifdef userodata}
          '.text','.data','.data','.rodata','.bss','.threadvar',
{$else userodata}
          '.text','.data','.data','.data','.bss','.threadvar',
{$endif userodata}
          '.pdata',
          '.text', { darwin stubs }
          '__DATA,__nl_symbol_ptr',
          '__DATA,__la_symbol_ptr',
          '__DATA,__mod_init_func',
          '__DATA,__mod_term_func',
          '.stab','.stabstr',
          '.idata$2','.idata$4','.idata$5','.idata$6','.idata$7','.edata',
          '.eh_frame',
          '.debug_frame','.debug_info','.debug_line','.debug_abbrev',
          '.fpc',
          '.toc',
          '.init',
          '.fini',
          '.objc_class',
          '.objc_meta_class',
          '.objc_cat_cls_meth',
          '.objc_cat_inst_meth',
          '.objc_protocol',
          '.objc_string_object',
          '.objc_cls_meth',
          '.objc_inst_meth',
          '.objc_cls_refs',
          '.objc_message_refs',
          '.objc_symbols',
          '.objc_category',
          '.objc_class_vars',
          '.objc_instance_vars',
          '.objc_module_info',
          '.objc_class_names',
          '.objc_meth_var_types',
          '.objc_meth_var_names',
          '.objc_selector_strs',
          '.objc_protocol_ext',
          '.objc_class_ext',
          '.objc_property',
          '.objc_image_info',
          '.objc_cstring_object',
          '.objc_sel_fixup',
          '__DATA,__objc_data',
          '__DATA,__objc_const',
          '.objc_superrefs',
          '__DATA, __datacoal_nt,coalesced',
          '.objc_classlist',
          '.objc_nlclasslist',
          '.objc_catlist',
          '.obcj_nlcatlist',
          '.objc_protolist'
        );
        secnames_pic : array[TAsmSectiontype] of string[length('__DATA, __datacoal_nt,coalesced')] = ('','',
          '.text',
          '.data.rel',
          '.data.rel',
          '.data.rel',
          '.bss',
          '.threadvar',
          '.pdata',
          '', { stubs }
          '__DATA,__nl_symbol_ptr',
          '__DATA,__la_symbol_ptr',
          '__DATA,__mod_init_func',
          '__DATA,__mod_term_func',
          '.stab',
          '.stabstr',
          '.idata$2','.idata$4','.idata$5','.idata$6','.idata$7','.edata',
          '.eh_frame',
          '.debug_frame','.debug_info','.debug_line','.debug_abbrev',
          '.fpc',
          '.toc',
          '.init',
          '.fini',
          '.objc_class',
          '.objc_meta_class',
          '.objc_cat_cls_meth',
          '.objc_cat_inst_meth',
          '.objc_protocol',
          '.objc_string_object',
          '.objc_cls_meth',
          '.objc_inst_meth',
          '.objc_cls_refs',
          '.objc_message_refs',
          '.objc_symbols',
          '.objc_category',
          '.objc_class_vars',
          '.objc_instance_vars',
          '.objc_module_info',
          '.objc_class_names',
          '.objc_meth_var_types',
          '.objc_meth_var_names',
          '.objc_selector_strs',
          '.objc_protocol_ext',
          '.objc_class_ext',
          '.objc_property',
          '.objc_image_info',
          '.objc_cstring_object',
          '.objc_sel_fixup',
          '__DATA,__objc_data',
          '__DATA,__objc_const',
          '.objc_superrefs',
          '__DATA, __datacoal_nt,coalesced',
          '.objc_classlist',
          '.objc_nlclasslist',
          '.objc_catlist',
          '.obcj_nlcatlist',
          '.objc_protolist'
        );
      var
        sep : string[3];
        secname : string;
      begin
        { section type user gives the user full controll on the section name }
        if atype=sec_user then
          result:=aname
        else
          begin
            if (cs_create_pic in current_settings.moduleswitches) and
               not(target_info.system in systems_darwin) then
              secname:=secnames_pic[atype]
            else
              secname:=secnames[atype];
            if (atype=sec_fpc) and (Copy(aname,1,3)='res') then
              begin
                result:=secname+'.'+aname;
                exit;
              end;
            if create_smartlink_sections and (aname<>'') then
              begin
                case aorder of
                  secorder_begin :
                    sep:='.b_';
                  secorder_end :
                    sep:='.z_';
                  else
                    sep:='.n_';
                end;
                result:=secname+sep+aname
              end
            else
              result:=secname;
          end;
      end;


    procedure TElfObjData.CreateDebugSections;
      begin
        if target_dbg.id=dbg_stabs then
          begin
            stabssec:=createsection(sec_stab);
            stabstrsec:=createsection(sec_stabstr);
          end;
      end;


    procedure TElfObjData.writereloc(data:aint;len:aword;p:TObjSymbol;reltype:TObjRelocationType);
      var
        symaddr : aint;
        objreloc: TObjRelocation;
      begin
        if CurrObjSec=nil then
          internalerror(200403292);
        objreloc:=nil;
        if assigned(p) then
         begin
           { real address of the symbol }
           symaddr:=p.address;
           { Local ObjSymbols can be resolved already or need a section reloc }
           if (p.bind=AB_LOCAL) and
              (reltype in [RELOC_RELATIVE,RELOC_ABSOLUTE{$ifdef x86_64},RELOC_ABSOLUTE32{$endif x86_64}]) then
             begin
               { For a reltype relocation in the same section the
                 value can be calculated }
               if (p.objsection=CurrObjSec) and
                  (reltype=RELOC_RELATIVE) then
                 inc(data,symaddr-len-CurrObjSec.Size)
               else
                 begin
                   objreloc:=TObjRelocation.CreateSection(CurrObjSec.Size,p.objsection,reltype);
                   CurrObjSec.ObjRelocations.Add(objreloc);
                   inc(data,symaddr);
                 end;
             end
           else
             begin
               objreloc:=TObjRelocation.CreateSymbol(CurrObjSec.Size,p,reltype);
               CurrObjSec.ObjRelocations.Add(objreloc);
               { If target is a local label and it isn't handled above,
                 patch its type in order to get it written to symtable.
                 This may happen e.g. when taking address of Pascal label in PIC mode. }
               if (p.bind=AB_LOCAL) and (p.typ=AT_LABEL) then
                 p.typ:=AT_ADDR;
            end;
         end;
        if assigned(objreloc) then
          begin
            objreloc.size:=len;
            if reltype in [RELOC_RELATIVE{$ifdef x86},RELOC_PLT32{$endif}{$ifdef x86_64},RELOC_GOTPCREL{$endif}] then
              dec(data,len);
            if relocs_use_addend then
              begin
                objreloc.orgsize:=data;
                data:=0;
              end;
          end;
        CurrObjSec.write(data,len);
      end;


{****************************************************************************
                            TElfSymtab
****************************************************************************}

    const
      symsecnames: array[boolean] of string[8] = ('.symtab','.dynsym');
      strsecnames: array[boolean] of string[8] = ('.strtab','.dynstr');
      symsectypes: array[boolean] of longint   = (SHT_SYMTAB,SHT_DYNSYM);
      symsecattrs: array[boolean] of longint   = (0,SHF_ALLOC);


    constructor TElfSymtab.create(aObjData:TObjData;aKind:TElfSymtabKind);
      var
        dyn:boolean;
      begin
        dyn:=(aKind=esk_dyn);
        create_ext(aObjData,symsecnames[dyn],symsectypes[dyn],symsecattrs[dyn],sizeof(pint),sizeof(TElfSymbol));
        fstrsec:=TElfObjSection.create_ext(aObjData,strsecnames[dyn],SHT_STRTAB,symsecattrs[dyn],1,0);
        fstrsec.writestr(#0);
        writezeros(sizeof(TElfSymbol));
        symidx:=1;
        shinfo:=1;
        kind:=aKind;
      end;

    procedure TElfSymtab.writeInternalSymbol(avalue:aword;astridx:longword;ainfo:byte;ashndx:word);
      var
        elfsym:TElfSymbol;
      begin
        fillchar(elfsym,sizeof(elfsym),0);
        elfsym.st_value:=avalue;
        elfsym.st_name:=astridx;
        elfsym.st_info:=ainfo;
        elfsym.st_shndx:=ashndx;
        inc(symidx);
        inc(shinfo);
        MaybeSwapElfSymbol(elfsym);
        write(elfsym,sizeof(elfsym));
      end;

    procedure TElfSymtab.writeSymbol(objsym:TObjSymbol;nameidx:longword);
      var
        elfsym:TElfSymbol;
      begin
        fillchar(elfsym,sizeof(elfsym),0);
        { symbolname, write the #0 separate to overcome 255+1 char not possible }
        if nameidx=0 then
          begin
            elfsym.st_name:=fstrsec.writestr(objsym.name);
            fstrsec.writestr(#0);
          end
        else
          elfsym.st_name:=nameidx;
        elfsym.st_size:=objsym.size;
        case objsym.bind of
          AB_LOCAL :
            begin
              elfsym.st_value:=objsym.address;
              elfsym.st_info:=STB_LOCAL shl 4;
              inc(shinfo);
            end;
          AB_COMMON :
            begin
              elfsym.st_value:=$10;            { ?? should not be hardcoded }
              elfsym.st_info:=STB_GLOBAL shl 4;
              elfsym.st_shndx:=SHN_COMMON;
            end;
          AB_EXTERNAL :
            elfsym.st_info:=STB_GLOBAL shl 4;
          AB_WEAK_EXTERNAL :
            elfsym.st_info:=STB_WEAK shl 4;
          AB_GLOBAL :
            begin
              elfsym.st_value:=objsym.address;
              elfsym.st_info:=STB_GLOBAL shl 4;
            end;
        end;
        if (objsym.bind<>AB_EXTERNAL) {and
           not(assigned(objsym.objsection) and
           not(oso_data in objsym.objsection.secoptions))} then
          begin
            case objsym.typ of
              AT_FUNCTION :
                elfsym.st_info:=elfsym.st_info or STT_FUNC;
              AT_DATA :
                elfsym.st_info:=elfsym.st_info or STT_OBJECT;
            end;
          end;
        if objsym.bind<>AB_COMMON then
          begin
            if kind<>esk_obj then
              begin
                { TODO }
              end
            else
              begin
                if assigned(objsym.objsection) then
                  elfsym.st_shndx:=objsym.objsection.index
                else
                  elfsym.st_shndx:=SHN_UNDEF;
                objsym.symidx:=symidx;
              end;
          end;
        inc(symidx);
        MaybeSwapElfSymbol(elfsym);
        write(elfsym,sizeof(TElfSymbol));
      end;

{****************************************************************************
                            TElfObjectOutput
****************************************************************************}

    constructor TElfObjectOutput.create(AWriter:TObjectWriter);
      begin
        inherited Create(AWriter);
        CObjData:=TElfObjData;
      end;


    procedure TElfObjectOutput.createrelocsection(s:TElfObjSection;data:TObjData);
      var
        i    : longint;
        rel  : telfreloc;
        objreloc : TObjRelocation;
        relsym   : longint;
        relocsect : TElfObjSection;
      begin
        { create the reloc section }
        relocsect:=TElfObjSection.create_reloc(data,s.name,false);
        relocsect.shlink:=symtabsect.index;
        relocsect.shinfo:=s.index;
        { add the relocations }
        for i:=0 to s.Objrelocations.count-1 do
          begin
            objreloc:=TObjRelocation(s.Objrelocations[i]);

            { Symbol }
            if assigned(objreloc.symbol) then
              begin
                if objreloc.symbol.symidx=-1 then
                  begin
                    writeln(objreloc.symbol.Name);
                    internalerror(200603012);
                  end;
                relsym:=objreloc.symbol.symidx;
              end
            else
              begin
                if objreloc.objsection<>nil then
                  relsym:=objreloc.objsection.secsymidx
                else
                  relsym:=SHN_UNDEF;
              end;

            rel.address:=objreloc.dataoffset;
            rel.info:=ELF_R_INFO(relsym,encodereloc(objreloc));
            rel.addend:=objreloc.orgsize;

            { write reloc }
            { ElfXX_Rel is essentially ElfXX_Rela without the addend field. }
            MaybeSwapElfReloc(rel);
            relocsect.write(rel,relocsect.shentsize);
          end;
      end;


    procedure TElfObjectOutput.section_write_symbol(p:TObject;arg:pointer);
      begin
        { Must not write symbols for internal sections like .symtab }
        { TODO: maybe use inclusive list of section types instead }
        if (TElfObjSection(p).shtype in [SHT_SYMTAB,SHT_STRTAB,SHT_REL,SHT_RELA]) then
          exit;
        TObjSection(p).secsymidx:=symtabsect.symidx;
        symtabsect.writeInternalSymbol(0,0,STT_SECTION,TObjSection(p).index);
      end;


    procedure TElfObjectOutput.createsymtab(data: TObjData);
      var
        i      : longint;
        objsym : TObjSymbol;
      begin
        with data do
         begin
           { filename entry }
           symtabsect.writeInternalSymbol(0,1,STT_FILE,SHN_ABS);
           { section }
           ObjSectionList.ForEachCall(@section_write_symbol,nil);
           { First the Local Symbols, this is required by ELF. The localsyms
             count stored in shinfo is used to skip the local symbols
             when traversing the symtab }
           for i:=0 to ObjSymbolList.Count-1 do
             begin
               objsym:=TObjSymbol(ObjSymbolList[i]);
               if (objsym.bind=AB_LOCAL) and (objsym.typ<>AT_LABEL) then
                 symtabsect.WriteSymbol(objsym);
             end;
           { Global Symbols }
           for i:=0 to ObjSymbolList.Count-1 do
             begin
               objsym:=TObjSymbol(ObjSymbolList[i]);
               if (objsym.bind<>AB_LOCAL) then
                 symtabsect.WriteSymbol(objsym);
             end;
           { update the .symtab section header }
           symtabsect.shlink:=symtabsect.fstrsec.index;
         end;
      end;


    procedure TElfObjectOutput.section_write_sh_string(p:TObject;arg:pointer);
      begin
        TElfObjSection(p).shstridx:=shstrtabsect.writestr(TObjSection(p).name+#0);
      end;


    procedure TElfObjectOutput.createshstrtab(data: TObjData);
      begin
        with data do
         begin
           shstrtabsect.writestr(#0);
           ObjSectionList.ForEachCall(@section_write_sh_string,nil);
         end;
      end;


    procedure TElfObjectOutput.writesectionheader(s:TElfObjSection);
      var
        sechdr : telfsechdr;
      begin
        fillchar(sechdr,sizeof(sechdr),0);
        sechdr.sh_name:=s.shstridx;
        sechdr.sh_type:=s.shtype;
        sechdr.sh_flags:=s.shflags;
        sechdr.sh_offset:=s.datapos;
        sechdr.sh_size:=s.Size;
        sechdr.sh_link:=s.shlink;
        sechdr.sh_info:=s.shinfo;
        sechdr.sh_addralign:=s.secalign;
        sechdr.sh_entsize:=s.shentsize;
        MaybeSwapSecHeader(sechdr);
        writer.write(sechdr,sizeof(sechdr));
      end;


    procedure TElfObjectOutput.section_count_sections(p:TObject;arg:pointer);
      begin
        TElfObjSection(p).index:=pword(arg)^;
        inc(pword(arg)^);
      end;


    procedure TElfObjectOutput.section_create_relocsec(p:TObject;arg:pointer);
      begin
        if (TElfObjSection(p).ObjRelocations.count>0) then
          createrelocsection(TElfObjSection(p),TObjData(arg));
      end;


    procedure TElfObjectOutput.section_write_sechdr(p:TObject;arg:pointer);
      begin
        writesectionheader(TElfObjSection(p));
      end;


    function TElfObjectOutput.writedata(data:TObjData):boolean;
      var
        header : telfheader;
        shoffset,
        datapos   : aword;
        nsections : word;
      begin
        result:=false;
        with data do
         begin
           { default sections }
           symtabsect:=TElfSymtab.create(data,esk_obj);
           shstrtabsect:=TElfObjSection.create_ext(data,'.shstrtab',SHT_STRTAB,0,1,0);
           { "no executable stack" marker for Linux }
           if (target_info.system in systems_linux) and
              not(cs_executable_stack in current_settings.moduleswitches) then
             TElfObjSection.create_ext(data,'.note.GNU-stack',SHT_PROGBITS,0,1,0);
           { insert filename as first in strtab }
           symtabsect.fstrsec.writestr(ExtractFileName(current_module.mainsource));
           symtabsect.fstrsec.writestr(#0);
           { calc amount of sections we have }
           nsections:=1;
           { also create the index in the section header table }
           ObjSectionList.ForEachCall(@section_count_sections,@nsections);
           { create .symtab and .strtab }
           createsymtab(data);
           { Create the relocation sections, this needs valid secidx and symidx }
           ObjSectionList.ForEachCall(@section_create_relocsec,data);
           { recalc nsections to incude the reloc sections }
           nsections:=1;
           ObjSectionList.ForEachCall(@section_count_sections,@nsections);
           { create .shstrtab }
           createshstrtab(data);

           { Calculate the filepositions }
           datapos:=$40; { elfheader + alignment }
           { section data }
           layoutsections(datapos);
           { section headers }
           shoffset:=datapos;
           inc(datapos,(nsections+1)*sizeof(telfsechdr));

           { Write ELF Header }
           fillchar(header,sizeof(header),0);
           header.magic[0]:=$7f; { = #127'ELF' }
           header.magic[1]:=$45;
           header.magic[2]:=$4c;
           header.magic[3]:=$46;
           header.file_class:=ELFCLASS;
           if target_info.endian=endian_big then
             header.data_encoding:=2
           else
             header.data_encoding:=1;

           header.file_version:=1;
           header.e_type:=ET_REL;
           header.e_machine:=ELFMACHINE;
{$ifdef arm}
           if (current_settings.fputype=fpu_soft) then
             header.e_flags:=$600;
{$endif arm}
           header.e_version:=1;
           header.e_shoff:=shoffset;
           header.e_shstrndx:=shstrtabsect.index;

           header.e_shnum:=nsections;
           header.e_ehsize:=sizeof(telfheader);
           header.e_shentsize:=sizeof(telfsechdr);
           MaybeSwapHeader(header);
           writer.write(header,sizeof(header));
           writer.writezeros($40-sizeof(header)); { align }
           { Sections }
           WriteSectionContent(data);
           { section headers, start with an empty header for sh_undef }
           writer.writezeros(sizeof(telfsechdr));
           ObjSectionList.ForEachCall(@section_write_sechdr,nil);
         end;
        result:=true;
      end;


end.

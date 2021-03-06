{
    Common subexpression elimination on base blocks

    Copyright (c) 2005-2012 by Florian Klaempfl

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
unit optcse;

{$i fpcdefs.inc}

{ $define csedebug}
{ $define csestats}

  interface

    uses
      node;

    {
      the function  creates non optimal code so far:
      - call para nodes are cse barriers because they can be reordered and thus the
        temp. creation could be done too late
      - the cse knows nothing about register pressure. In case of high register pressure, cse might
        have a negative impact
      - the list of cseinvariant node types and inline numbers is not complete yet

      Further, it could be done probably in a faster way though the complexity can't probably not reduced
    }
    function do_optcse(var rootnode : tnode) : tnode;

  implementation

    uses
      globtype,globals,
      cclasses,
      verbose,
      nutils,
      procinfo,
      nbas,nld,ninl,ncal,ncnv,nadd,nmem,
      pass_1,
      symconst,symtype,symdef,symsym,
      defutil,
      optbase;

    const
      cseinvariant : set of tnodetype = [addn,muln,subn,divn,slashn,modn,andn,orn,xorn,notn,vecn,
        derefn,equaln,unequaln,ltn,gtn,lten,gten,typeconvn,subscriptn,
        inn,symdifn,shrn,shln,ordconstn,realconstn,unaryminusn,pointerconstn,stringconstn,setconstn,niln,
        setelementn,{arrayconstructorn,arrayconstructorrangen,}
        isn,asn,starstarn,nothingn,temprefn,loadparentfpn {,callparan},assignn];

    function searchsubdomain(var n:tnode; arg: pointer) : foreachnoderesult;
      begin
        if (n.nodetype in cseinvariant) or
          ((n.nodetype=inlinen) and
           (tinlinenode(n).inlinenumber in [in_assigned_x])
          ) or
          ((n.nodetype=callparan) and not(assigned(tcallparanode(n).right))) or
          ((n.nodetype=loadn) and
            not((tloadnode(n).symtableentry.typ in [staticvarsym,localvarsym,paravarsym]) and
                (vo_volatile in tabstractvarsym(tloadnode(n).symtableentry).varoptions))
          ) then
          result:=fen_true
        else
          begin
            pboolean(arg)^:=false;
            result:=fen_norecurse_true;
          end;
      end;

    type
      tlists = record
        nodelist : tfplist;
        locationlist : tfplist;
        equalto : tfplist;
        refs : tfplist;
        avail : TDFASet;
      end;

      plists = ^tlists;

    { collectnodes needs the address of itself to call foreachnodestatic,
      so we need a wrapper because @<func> inside <func doesn't work }

    function collectnodes(var n:tnode; arg: pointer) : foreachnoderesult;forward;

    function collectnodes2(var n:tnode; arg: pointer) : foreachnoderesult;
      begin
        result:=collectnodes(n,arg);
      end;

    function collectnodes(var n:tnode; arg: pointer) : foreachnoderesult;
      var
        i,j : longint;
      begin
        result:=fen_false;
        { don't add the tree below an untyped const parameter: there is
          no information available that this kind of tree actually needs
          to be addresable, this could be improved }
        if ((n.nodetype=callparan) and
          (tcallparanode(n).left.resultdef.typ=formaldef) and
          (tcallparanode(n).parasym.varspez=vs_const)) then
          begin
            result:=fen_norecurse_false;
            exit;
          end;
        if
          { node possible to add? }
          assigned(n.resultdef) and
          (
            { regable expressions }
            (n.actualtargetnode.flags*[nf_write,nf_modify]=[]) and
            ((tstoreddef(n.resultdef).is_intregable or tstoreddef(n.resultdef).is_fpuregable) and
            { is_int/fpuregable allows arrays and records to be in registers, cse cannot handle this }
            (not(n.resultdef.typ in [arraydef,recorddef])) and
            { same for voiddef }
            not(is_void(n.resultdef)) and
            { adding tempref and callpara nodes itself is worthless but
              their complexity is probably <= 1 anyways }
            not(n.nodetype in [temprefn,callparan]) and

            { node worth to add?

              We consider almost every node because even loading a variables from
              a register instead of memory is more beneficial. This behaviour should
              not increase register pressure because if a variable is already
              in a register, the reg. allocator can merge the nodes. If a variable
              is loaded from memory, loading this variable and spilling another register
              should not add a speed penalty.
            }
            {
              load nodes are not considered if they load para or local symbols from the
              current stack frame, those are in registers anyways if possible
            }
            (not(n.nodetype=loadn) or
             not(tloadnode(n).symtableentry.typ in [paravarsym,localvarsym]) or
             (node_complexity(n)>1)
            ) and

            {
              Const nodes however are only considered if their complexity is >1
              This might be the case for the risc architectures if they need
              more than one instruction to load this particular value
            }
            (not(is_constnode(n)) or (node_complexity(n)>1)))
{$ifndef x86}
            or
            { store reference of expression? }

            { loading the address of a global symbol takes typically more than
              one instruction on every platform except x86
              so consider in this case loading the address of the data
            }
            (((n.resultdef.typ in [arraydef,recorddef]) or is_object(n.resultdef)) and
             (n.nodetype=loadn) and
             (tloadnode(n).symtableentry.typ=staticvarsym)
            )
{$endif x86}
          ) then
          begin
            plists(arg)^.nodelist.Add(n);
            plists(arg)^.locationlist.Add(@n);
            plists(arg)^.refs.Add(nil);
            plists(arg)^.equalto.Add(pointer(-1));

            DFASetInclude(plists(arg)^.avail,plists(arg)^.nodelist.count-1);

            for i:=0 to plists(arg)^.nodelist.count-2 do
              begin
                if tnode(plists(arg)^.nodelist[i]).isequal(n) and DFASetIn(plists(arg)^.avail,i) then
                  begin
                    { use always the first occurence }
                    if plists(arg)^.equalto[i]<>pointer(-1) then
                      plists(arg)^.equalto[plists(arg)^.nodelist.count-1]:=plists(arg)^.equalto[i]
                    else
                      plists(arg)^.equalto[plists(arg)^.nodelist.count-1]:=pointer(ptrint(i));
                    plists(arg)^.refs[i]:=pointer(plists(arg)^.refs[i])+1;
                    break;
                  end;
              end;

            { boolean and/or require a special handling: after evaluating the and/or node,
              the expressions of the right side might not be available due to short boolean
              evaluation, so after handling the right side, mark those expressions
              as unavailable }
            if (n.nodetype in [orn,andn]) and is_boolean(taddnode(n).left.resultdef) then
              begin
                foreachnodestatic(pm_postprocess,taddnode(n).left,@collectnodes2,arg);
                j:=plists(arg)^.nodelist.count;
                foreachnodestatic(pm_postprocess,taddnode(n).right,@collectnodes2,arg);
                for i:=j to plists(arg)^.nodelist.count-1 do
                  DFASetExclude(plists(arg)^.avail,i);
                result:=fen_norecurse_false;
              end;
          end;
      end;


    function searchcsedomain(var n: tnode; arg: pointer) : foreachnoderesult;
      var
        csedomain : boolean;
        lists : tlists;
        templist : tfplist;
        i : longint;
        def : tstoreddef;
        nodes : tblocknode;
        creates,
        statements : tstatementnode;
        hp : ttempcreatenode;
        addrstored : boolean;
        hp2 : tnode;
      begin
        result:=fen_false;
        if n.nodetype in cseinvariant then
          begin
            csedomain:=true;
            foreachnodestatic(pm_postprocess,n,@searchsubdomain,@csedomain);
            if not(csedomain) then
              begin
                { try to transform the tree to get better cse domains, consider:
                       +
                      / \
                     +   C
                    / \
                   A   B

                  if A is not cse'able but B and C are, then the compiler cannot do cse so the tree is transformed into
                       +
                      / \
                     A   +
                        / \
                       B   C
                  Because A could be another tree of this kind, the whole process is done in a while loop
                }
                if (n.nodetype in [andn,orn,addn,muln]) and
                  (n.nodetype=tbinarynode(n).left.nodetype) and
                  { do is optimizations only for integers, reals (no currency!), vectors, sets or booleans }
                  (is_integer(n.resultdef) or is_real(n.resultdef) or is_vector(n.resultdef) or is_set(n.resultdef) or
                   is_boolean(n.resultdef)) and
                  { either if fastmath is on }
                  ((cs_opt_fastmath in current_settings.optimizerswitches) or
                   { or for the logical operators, they cannot overflow }
                   (n.nodetype in [andn,orn]) or
                   { or for integers if range checking is off }
                   ((is_integer(n.resultdef) and
                    (n.localswitches*[cs_check_range,cs_check_overflow]=[]) and
                    (tbinarynode(n).left.localswitches*[cs_check_range,cs_check_overflow]=[]))) or
                   { for sets, we can do this always }
                   (is_set(n.resultdef))
                   ) then
                  while n.nodetype=tbinarynode(n).left.nodetype do
                    begin
                      csedomain:=true;
                      foreachnodestatic(pm_postprocess,tbinarynode(n).right,@searchsubdomain,@csedomain);
                      if csedomain then
                        begin
                          csedomain:=true;
                          foreachnodestatic(pm_postprocess,tbinarynode(tbinarynode(n).left).right,@searchsubdomain,@csedomain);
                          if csedomain then
                            begin
                              hp2:=tbinarynode(tbinarynode(n).left).left;
                              tbinarynode(tbinarynode(n).left).left:=tbinarynode(tbinarynode(n).left).right;
                              tbinarynode(tbinarynode(n).left).right:=tbinarynode(n).right;
                              tbinarynode(n).right:=tbinarynode(n).left;
                              tbinarynode(n).left:=hp2;

                              { the transformed tree could result in new possibilities to fold constants
                                so force a firstpass on the root node }
                              exclude(tbinarynode(n).right.flags,nf_pass1_done);
                              do_firstpass(tbinarynode(n).right);
                            end
                          else
                            break;
                        end
                      else
                        break;
                    end;
              end
            else
              begin
                statements:=nil;
                result:=fen_norecurse_true;
{$ifdef csedebug}
                writeln('============ cse domain ==================');
                printnode(output,n);
                writeln('Complexity: ',node_complexity(n));
{$endif csedebug}
                lists.nodelist:=tfplist.create;
                lists.locationlist:=tfplist.create;
                lists.equalto:=tfplist.create;
                lists.refs:=tfplist.create;
                foreachnodestatic(pm_postprocess,n,@collectnodes,@lists);

                templist:=tfplist.create;
                templist.count:=lists.nodelist.count;

                { check all nodes if one is used more than once }
                for i:=0 to lists.nodelist.count-1 do
                  begin
                    { current node used more than once? }
                    if assigned(lists.refs[i]) then
                      begin
                        if not(assigned(statements)) then
                          begin
                            nodes:=internalstatements(statements);
                            addstatement(statements,internalstatements(creates));
                          end;

                        def:=tstoreddef(tnode(lists.nodelist[i]).resultdef);
                        { we cannot handle register stored records or array in CSE yet
                          but we can store their reference }
                        addrstored:=(def.typ in [arraydef,recorddef]) or is_object(def);

                        if addrstored then
                          templist[i]:=ctempcreatenode.create_value(getpointerdef(def),voidpointertype.size,tt_persistent,
                            true,caddrnode.create(tnode(lists.nodelist[i])))
                        else
                          templist[i]:=ctempcreatenode.create_value(def,def.size,tt_persistent,
                            def.is_intregable or def.is_fpuregable,tnode(lists.nodelist[i]));

                        { make debugging easier and set temp. location to the original location }
                        tnode(templist[i]).fileinfo:=tnode(lists.nodelist[i]).fileinfo;

                        addstatement(creates,tnode(templist[i]));
                        { make debugging easier and set temp. location to the original location }
                        creates.fileinfo:=tnode(lists.nodelist[i]).fileinfo;

                        hp:=ttempcreatenode(templist[i]);
                        do_firstpass(tnode(hp));
                        templist[i]:=hp;

                        if addrstored then
                          pnode(lists.locationlist[i])^:=cderefnode.Create(ctemprefnode.create(ttempcreatenode(templist[i])))
                        else
                          pnode(lists.locationlist[i])^:=ctemprefnode.create(ttempcreatenode(templist[i]));
                        { make debugging easier and set temp. location to the original location }
                        pnode(lists.locationlist[i])^.fileinfo:=tnode(lists.nodelist[i]).fileinfo;

                        do_firstpass(pnode(lists.locationlist[i])^);
{$ifdef csedebug}
                        printnode(output,statements);
{$endif csedebug}
                      end
                    { current node reference to another node? }
                    else if lists.equalto[i]<>pointer(-1) then
                      begin
                        def:=tstoreddef(tnode(lists.nodelist[i]).resultdef);
                        { we cannot handle register stored records or array in CSE yet
                          but we can store their reference }
                        addrstored:=(def.typ in [arraydef,recorddef]) or is_object(def);

{$if defined(csedebug) or defined(csestats)}
                        writeln;
                        writeln('!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!');
                        writeln('Complexity: ',node_complexity(tnode(lists.nodelist[i])),'  Node ',i,' equals Node ',ptrint(lists.equalto[i]));
                        printnode(output,tnode(lists.nodelist[i]));
                        printnode(output,tnode(lists.nodelist[ptrint(lists.equalto[i])]));
                        writeln('!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!');
                        writeln;
{$endif defined(csedebug) or defined(csestats)}
                        templist[i]:=templist[ptrint(lists.equalto[i])];
                        if addrstored then
                          pnode(lists.locationlist[i])^:=cderefnode.Create(ctemprefnode.create(ttempcreatenode(templist[ptrint(lists.equalto[i])])))
                        else
                          pnode(lists.locationlist[i])^:=ctemprefnode.create(ttempcreatenode(templist[ptrint(lists.equalto[i])]));

                        { make debugging easier and set temp. location to the original location }
                        pnode(lists.locationlist[i])^.fileinfo:=tnode(lists.nodelist[i]).fileinfo;

                        do_firstpass(pnode(lists.locationlist[i])^);
                      end;
                  end;
                { clean up unused trees }
                for i:=0 to lists.nodelist.count-1 do
                  if lists.equalto[i]<>pointer(-1) then
                    tnode(lists.nodelist[i]).free;
{$ifdef csedebug}
                writeln('nodes: ',lists.nodelist.count);
                writeln('==========================================');
{$endif csedebug}
                lists.nodelist.free;
                lists.locationlist.free;
                lists.equalto.free;
                lists.refs.free;
                templist.free;

                if assigned(statements) then
                  begin
                    { call para nodes need a special handling because
                      they can be only children nodes of call nodes
                      so the initialization code is inserted below the
                      call para node
                    }
                    if n.nodetype=callparan then
                      begin
                        addstatement(statements,tcallparanode(n).left);
                        tcallparanode(n).left:=nodes;
                        do_firstpass(tcallparanode(n).left);
                      end
                    else
                      begin
                        addstatement(statements,n);
                        n:=nodes;
                        do_firstpass(n);
                      end;
{$ifdef csedebug}
                    printnode(output,nodes);
{$endif csedebug}
                  end;
              end
          end;
      end;


    function do_optcse(var rootnode : tnode) : tnode;
      begin
{$ifdef csedebug}
         writeln('====================================================================================');
         writeln('CSE optimization pass started');
         writeln('====================================================================================');
         printnode(rootnode);
         writeln('====================================================================================');
         writeln;
{$endif csedebug}
        foreachnodestatic(pm_postprocess,rootnode,@searchcsedomain,nil);
        result:=nil;
      end;

end.

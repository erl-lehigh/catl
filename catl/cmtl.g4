grammar cmtl;

@header {
'''
 Copyright (C) 2018 Cristian Ioan Vasile <cvasile@mit.edu>
 Hybrid and Networked Systems (HyNeSs) Group, BU Robotics Lab, Boston University
 See license.txt file for license information.
'''
}


cmtlProperty:
         '(' child=cmtlProperty ')' #parprop
    |    predicate #cmtlPredicate
    |    op=NOT child=cmtlProperty #formula
    |    op=EVENT '[' low=(RATIONAL | INT) ',' high=(RATIONAL | INT) ']'
         child=cmtlProperty #formula
    |    op=ALWAYS '[' low=(RATIONAL | INT) ',' high=(RATIONAL | INT) ']'
         child=cmtlProperty #formula
    |    left=cmtlProperty op=IMPLIES right=cmtlProperty #formula
    |    left=cmtlProperty op=AND right=cmtlProperty #formula
    |    left=cmtlProperty op=OR right=cmtlProperty #formula
    |    left=cmtlProperty
         op=UNTIL '[' low=(RATIONAL | INT) ',' high=(RATIONAL | INT) ']'
         right=cmtlProperty #formula
    ;
predicate:
         op='T' '(' duration=(RATIONAL | INT) ',' proposition=VARIABLE
        ',' capabilities ')'
    |    op=BOOLEAN
    ;
capabilities: '{' capabilityRequest ( ',' capabilityRequest )* '}'
    ;
capabilityRequest: '(' cap=VARIABLE ',' count=INT ')'
    ;

AND : '&' | '&&' | '/\\' ;
OR : '|' | '||' | '\\/' ;
IMPLIES : '=>' ;
NOT : '!' | '~' ;
EVENT : 'F' | '<>' ;
ALWAYS : 'G' | '[]' ;
UNTIL : 'U' ;
BOOLEAN : 'true' | 'True' | 'false' | 'False' ;
VARIABLE : ( [a-z] | [A-Z] )( [a-z] | [A-Z] | [0-9] | '_' )* ;
RATIONAL : INT ('.')? [0-9]+ ( 'E' | 'E-' )? [0-9]* ;
INT : '0' | ('-')? [1-9] [0-9]* ;
WS : ( ' ' | '\t' | '\r' | '\n' )+ -> skip ;

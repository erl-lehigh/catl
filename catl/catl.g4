grammar catl;

@header {
'''
 Copyright (C) 2018-2020 Cristian Ioan Vasile <cvasile@lehigh.edu>
 Explainable Robotics Lab (ERL), Autonomous and Intelligent Robotics (AIR) Lab,
 Lehigh University
 Hybrid and Networked Systems (HyNeSs) Group, BU Robotics Lab, Boston University
 See license.txt file for license information.
'''
}


catlProperty:
         '(' child=catlProperty ')' #parprop
    |    predicate #catlPredicate
    |    op=NOT child=catlProperty #formula
    |    op=EVENT '[' low=(RATIONAL | INT) ',' high=(RATIONAL | INT) ']'
         child=catlProperty #formula
    |    op=ALWAYS '[' low=(RATIONAL | INT) ',' high=(RATIONAL | INT) ']'
         child=catlProperty #formula
    |    left=catlProperty op=IMPLIES right=catlProperty #formula
    |    left=catlProperty op=AND right=catlProperty #formula
    |    left=catlProperty op=OR right=catlProperty #formula
    |    left=catlProperty
         op=UNTIL '[' low=(RATIONAL | INT) ',' high=(RATIONAL | INT) ']'
         right=catlProperty #formula
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

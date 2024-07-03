grammar stl;

@header {
'''
 Copyright (c) 2023, Explainable Robotics Lab (ERL)
 See license.txt file for license information.
 @author: Gustavo A. Cardona, Cristian-Ioan Vasile
'''
}


stlProperty:
         '(' child=stlProperty ')' #parprop
    |    booleanExpr #booleanPred
    |    op=NOT child=stlProperty #formula
    |    op=EVENT '[' low=RATIONAL ',' high=RATIONAL ']' child=stlProperty #formula
    |    op=EEVENT '[' low=RATIONAL ',' high=RATIONAL ']' child=stlProperty #formula
    |    op=ALWAYS '[' low=RATIONAL ',' high=RATIONAL ']' child=stlProperty #formula
    |    op=EALWAYS '[' low=RATIONAL ',' high=RATIONAL ']' child=stlProperty #formula
    |    left=stlProperty op=IMPLIES right=stlProperty #formula
    |    left=stlProperty op=AND right=stlProperty #formula
    |    left=stlProperty op=EAND right=stlProperty #formula    
    |    left=stlProperty op=OR right=stlProperty #formula
    |    left=stlProperty op=EOR right=stlProperty #formula
    |    left=stlProperty op=UNTIL '[' low=RATIONAL ',' high=RATIONAL ']' right=stlProperty #formula
    ;   
expr:
        ( '-(' | '(' ) expr ')'
    |   <assoc=right>   expr '^' expr
    |   VARIABLE '(' expr ')'
    |   expr ( '*' | '/' ) expr
    |   expr ( '+' | '-' ) expr
    |   RATIONAL
    |   VARIABLE
    ;
booleanExpr:
         left=expr op=( '<' | '<=' | '=' | '>=' | '>' ) right=expr
    |    op=BOOLEAN
    ;
AND : '&&' | '/\\' ;
EAND: '&' ;
OR :  '||' | '\\/' ;
EOR: '|' ;
IMPLIES : '=>' ;
NOT : '!' | '~' ;
EVENT : 'F' ;
EEVENT :  'E';
ALWAYS : 'G' ; 
EALWAYS : 'A' ;
UNTIL : 'U' ;
BOOLEAN : 'true' | 'True' | 'false' | 'False' ;
VARIABLE : ( [a-z] | [A-Z] )( [a-z] | [A-Z] | [0-9] | '_' )* ;
RATIONAL : ('-')? [0-9]* ('.')? [0-9]+ ( 'E' | 'E-' )? [0-9]* ;
WS : ( ' ' | '\t' | '\r' | '\n' )+ -> skip ;
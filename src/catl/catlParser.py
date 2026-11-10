# Generated from catl.g4 by ANTLR 4.13.0
# encoding: utf-8
from antlr4 import *
from io import StringIO
import sys
if sys.version_info[1] > 5:
	from typing import TextIO
else:
	from typing.io import TextIO


'''
 Copyright (C) 2018-2020 Cristian Ioan Vasile <cvasile@lehigh.edu>
 Explainable Robotics Lab (ERL), Autonomous and Intelligent Robotics (AIR) Lab,
 Lehigh University
 Hybrid and Networked Systems (HyNeSs) Group, BU Robotics Lab, Boston University
 See license.txt file for license information.
'''

def serializedATN():
    return [
        4,1,20,109,2,0,7,0,2,1,7,1,2,2,7,2,2,3,7,3,2,4,7,4,2,5,7,5,1,0,1,
        0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,
        0,1,0,1,0,1,0,1,0,3,0,35,8,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,
        1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,5,0,54,8,0,10,0,12,0,57,9,0,1,1,
        1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,3,1,68,8,1,1,1,1,1,1,1,3,1,73,8,
        1,1,2,1,2,1,2,1,2,5,2,79,8,2,10,2,12,2,82,9,2,1,2,1,2,1,3,1,3,1,
        3,1,3,1,3,1,3,1,4,1,4,1,4,1,4,5,4,96,8,4,10,4,12,4,99,9,4,1,4,1,
        4,1,5,1,5,1,5,1,5,1,5,1,5,1,5,0,1,0,6,0,2,4,6,8,10,0,1,1,0,18,19,
        114,0,34,1,0,0,0,2,72,1,0,0,0,4,74,1,0,0,0,6,85,1,0,0,0,8,91,1,0,
        0,0,10,102,1,0,0,0,12,13,6,0,-1,0,13,14,5,1,0,0,14,15,3,0,0,0,15,
        16,5,2,0,0,16,35,1,0,0,0,17,35,3,2,1,0,18,19,5,12,0,0,19,35,3,0,
        0,7,20,21,5,13,0,0,21,22,5,3,0,0,22,23,7,0,0,0,23,24,5,4,0,0,24,
        25,7,0,0,0,25,26,5,5,0,0,26,35,3,0,0,6,27,28,5,14,0,0,28,29,5,3,
        0,0,29,30,7,0,0,0,30,31,5,4,0,0,31,32,7,0,0,0,32,33,5,5,0,0,33,35,
        3,0,0,5,34,12,1,0,0,0,34,17,1,0,0,0,34,18,1,0,0,0,34,20,1,0,0,0,
        34,27,1,0,0,0,35,55,1,0,0,0,36,37,10,4,0,0,37,38,5,11,0,0,38,54,
        3,0,0,5,39,40,10,3,0,0,40,41,5,9,0,0,41,54,3,0,0,4,42,43,10,2,0,
        0,43,44,5,10,0,0,44,54,3,0,0,3,45,46,10,1,0,0,46,47,5,15,0,0,47,
        48,5,3,0,0,48,49,7,0,0,0,49,50,5,4,0,0,50,51,7,0,0,0,51,52,5,5,0,
        0,52,54,3,0,0,2,53,36,1,0,0,0,53,39,1,0,0,0,53,42,1,0,0,0,53,45,
        1,0,0,0,54,57,1,0,0,0,55,53,1,0,0,0,55,56,1,0,0,0,56,1,1,0,0,0,57,
        55,1,0,0,0,58,59,5,6,0,0,59,60,5,1,0,0,60,61,7,0,0,0,61,62,5,4,0,
        0,62,63,5,17,0,0,63,64,5,4,0,0,64,67,3,4,2,0,65,66,5,4,0,0,66,68,
        3,8,4,0,67,65,1,0,0,0,67,68,1,0,0,0,68,69,1,0,0,0,69,70,5,2,0,0,
        70,73,1,0,0,0,71,73,5,16,0,0,72,58,1,0,0,0,72,71,1,0,0,0,73,3,1,
        0,0,0,74,75,5,7,0,0,75,80,3,6,3,0,76,77,5,4,0,0,77,79,3,6,3,0,78,
        76,1,0,0,0,79,82,1,0,0,0,80,78,1,0,0,0,80,81,1,0,0,0,81,83,1,0,0,
        0,82,80,1,0,0,0,83,84,5,8,0,0,84,5,1,0,0,0,85,86,5,1,0,0,86,87,5,
        17,0,0,87,88,5,4,0,0,88,89,5,19,0,0,89,90,5,2,0,0,90,7,1,0,0,0,91,
        92,5,7,0,0,92,97,3,10,5,0,93,94,5,4,0,0,94,96,3,10,5,0,95,93,1,0,
        0,0,96,99,1,0,0,0,97,95,1,0,0,0,97,98,1,0,0,0,98,100,1,0,0,0,99,
        97,1,0,0,0,100,101,5,8,0,0,101,9,1,0,0,0,102,103,5,1,0,0,103,104,
        5,17,0,0,104,105,5,4,0,0,105,106,7,0,0,0,106,107,5,2,0,0,107,11,
        1,0,0,0,7,34,53,55,67,72,80,97
    ]

class catlParser ( Parser ):

    grammarFileName = "catl.g4"

    atn = ATNDeserializer().deserialize(serializedATN())

    decisionsToDFA = [ DFA(ds, i) for i, ds in enumerate(atn.decisionToState) ]

    sharedContextCache = PredictionContextCache()

    literalNames = [ "<INVALID>", "'('", "')'", "'['", "','", "']'", "'T'", 
                     "'{'", "'}'", "<INVALID>", "<INVALID>", "'=>'", "<INVALID>", 
                     "<INVALID>", "<INVALID>", "'U'" ]

    symbolicNames = [ "<INVALID>", "<INVALID>", "<INVALID>", "<INVALID>", 
                      "<INVALID>", "<INVALID>", "<INVALID>", "<INVALID>", 
                      "<INVALID>", "AND", "OR", "IMPLIES", "NOT", "EVENT", 
                      "ALWAYS", "UNTIL", "BOOLEAN", "VARIABLE", "RATIONAL", 
                      "INT", "WS" ]

    RULE_catlProperty = 0
    RULE_predicate = 1
    RULE_capabilities = 2
    RULE_capabilityRequest = 3
    RULE_resources = 4
    RULE_resourceRequest = 5

    ruleNames =  [ "catlProperty", "predicate", "capabilities", "capabilityRequest", 
                   "resources", "resourceRequest" ]

    EOF = Token.EOF
    T__0=1
    T__1=2
    T__2=3
    T__3=4
    T__4=5
    T__5=6
    T__6=7
    T__7=8
    AND=9
    OR=10
    IMPLIES=11
    NOT=12
    EVENT=13
    ALWAYS=14
    UNTIL=15
    BOOLEAN=16
    VARIABLE=17
    RATIONAL=18
    INT=19
    WS=20

    def __init__(self, input:TokenStream, output:TextIO = sys.stdout):
        super().__init__(input, output)
        self.checkVersion("4.13.0")
        self._interp = ParserATNSimulator(self, self.atn, self.decisionsToDFA, self.sharedContextCache)
        self._predicates = None




    class CatlPropertyContext(ParserRuleContext):
        __slots__ = 'parser'

        def __init__(self, parser, parent:ParserRuleContext=None, invokingState:int=-1):
            super().__init__(parent, invokingState)
            self.parser = parser


        def getRuleIndex(self):
            return catlParser.RULE_catlProperty

     
        def copyFrom(self, ctx:ParserRuleContext):
            super().copyFrom(ctx)


    class CatlPredicateContext(CatlPropertyContext):

        def __init__(self, parser, ctx:ParserRuleContext): # actually a catlParser.CatlPropertyContext
            super().__init__(parser)
            self.copyFrom(ctx)

        def predicate(self):
            return self.getTypedRuleContext(catlParser.PredicateContext,0)


        def enterRule(self, listener:ParseTreeListener):
            if hasattr( listener, "enterCatlPredicate" ):
                listener.enterCatlPredicate(self)

        def exitRule(self, listener:ParseTreeListener):
            if hasattr( listener, "exitCatlPredicate" ):
                listener.exitCatlPredicate(self)

        def accept(self, visitor:ParseTreeVisitor):
            if hasattr( visitor, "visitCatlPredicate" ):
                return visitor.visitCatlPredicate(self)
            else:
                return visitor.visitChildren(self)


    class FormulaContext(CatlPropertyContext):

        def __init__(self, parser, ctx:ParserRuleContext): # actually a catlParser.CatlPropertyContext
            super().__init__(parser)
            self.left = None # CatlPropertyContext
            self.op = None # Token
            self.child = None # CatlPropertyContext
            self.low = None # Token
            self.high = None # Token
            self.right = None # CatlPropertyContext
            self.copyFrom(ctx)

        def NOT(self):
            return self.getToken(catlParser.NOT, 0)
        def catlProperty(self, i:int=None):
            if i is None:
                return self.getTypedRuleContexts(catlParser.CatlPropertyContext)
            else:
                return self.getTypedRuleContext(catlParser.CatlPropertyContext,i)

        def EVENT(self):
            return self.getToken(catlParser.EVENT, 0)
        def RATIONAL(self, i:int=None):
            if i is None:
                return self.getTokens(catlParser.RATIONAL)
            else:
                return self.getToken(catlParser.RATIONAL, i)
        def INT(self, i:int=None):
            if i is None:
                return self.getTokens(catlParser.INT)
            else:
                return self.getToken(catlParser.INT, i)
        def ALWAYS(self):
            return self.getToken(catlParser.ALWAYS, 0)
        def IMPLIES(self):
            return self.getToken(catlParser.IMPLIES, 0)
        def AND(self):
            return self.getToken(catlParser.AND, 0)
        def OR(self):
            return self.getToken(catlParser.OR, 0)
        def UNTIL(self):
            return self.getToken(catlParser.UNTIL, 0)

        def enterRule(self, listener:ParseTreeListener):
            if hasattr( listener, "enterFormula" ):
                listener.enterFormula(self)

        def exitRule(self, listener:ParseTreeListener):
            if hasattr( listener, "exitFormula" ):
                listener.exitFormula(self)

        def accept(self, visitor:ParseTreeVisitor):
            if hasattr( visitor, "visitFormula" ):
                return visitor.visitFormula(self)
            else:
                return visitor.visitChildren(self)


    class ParpropContext(CatlPropertyContext):

        def __init__(self, parser, ctx:ParserRuleContext): # actually a catlParser.CatlPropertyContext
            super().__init__(parser)
            self.child = None # CatlPropertyContext
            self.copyFrom(ctx)

        def catlProperty(self):
            return self.getTypedRuleContext(catlParser.CatlPropertyContext,0)


        def enterRule(self, listener:ParseTreeListener):
            if hasattr( listener, "enterParprop" ):
                listener.enterParprop(self)

        def exitRule(self, listener:ParseTreeListener):
            if hasattr( listener, "exitParprop" ):
                listener.exitParprop(self)

        def accept(self, visitor:ParseTreeVisitor):
            if hasattr( visitor, "visitParprop" ):
                return visitor.visitParprop(self)
            else:
                return visitor.visitChildren(self)



    def catlProperty(self, _p:int=0):
        _parentctx = self._ctx
        _parentState = self.state
        localctx = catlParser.CatlPropertyContext(self, self._ctx, _parentState)
        _prevctx = localctx
        _startState = 0
        self.enterRecursionRule(localctx, 0, self.RULE_catlProperty, _p)
        self._la = 0 # Token type
        try:
            self.enterOuterAlt(localctx, 1)
            self.state = 34
            self._errHandler.sync(self)
            token = self._input.LA(1)
            if token in [1]:
                localctx = catlParser.ParpropContext(self, localctx)
                self._ctx = localctx
                _prevctx = localctx

                self.state = 13
                self.match(catlParser.T__0)
                self.state = 14
                localctx.child = self.catlProperty(0)
                self.state = 15
                self.match(catlParser.T__1)
                pass
            elif token in [6, 16]:
                localctx = catlParser.CatlPredicateContext(self, localctx)
                self._ctx = localctx
                _prevctx = localctx
                self.state = 17
                self.predicate()
                pass
            elif token in [12]:
                localctx = catlParser.FormulaContext(self, localctx)
                self._ctx = localctx
                _prevctx = localctx
                self.state = 18
                localctx.op = self.match(catlParser.NOT)
                self.state = 19
                localctx.child = self.catlProperty(7)
                pass
            elif token in [13]:
                localctx = catlParser.FormulaContext(self, localctx)
                self._ctx = localctx
                _prevctx = localctx
                self.state = 20
                localctx.op = self.match(catlParser.EVENT)
                self.state = 21
                self.match(catlParser.T__2)
                self.state = 22
                localctx.low = self._input.LT(1)
                _la = self._input.LA(1)
                if not(_la==18 or _la==19):
                    localctx.low = self._errHandler.recoverInline(self)
                else:
                    self._errHandler.reportMatch(self)
                    self.consume()
                self.state = 23
                self.match(catlParser.T__3)
                self.state = 24
                localctx.high = self._input.LT(1)
                _la = self._input.LA(1)
                if not(_la==18 or _la==19):
                    localctx.high = self._errHandler.recoverInline(self)
                else:
                    self._errHandler.reportMatch(self)
                    self.consume()
                self.state = 25
                self.match(catlParser.T__4)
                self.state = 26
                localctx.child = self.catlProperty(6)
                pass
            elif token in [14]:
                localctx = catlParser.FormulaContext(self, localctx)
                self._ctx = localctx
                _prevctx = localctx
                self.state = 27
                localctx.op = self.match(catlParser.ALWAYS)
                self.state = 28
                self.match(catlParser.T__2)
                self.state = 29
                localctx.low = self._input.LT(1)
                _la = self._input.LA(1)
                if not(_la==18 or _la==19):
                    localctx.low = self._errHandler.recoverInline(self)
                else:
                    self._errHandler.reportMatch(self)
                    self.consume()
                self.state = 30
                self.match(catlParser.T__3)
                self.state = 31
                localctx.high = self._input.LT(1)
                _la = self._input.LA(1)
                if not(_la==18 or _la==19):
                    localctx.high = self._errHandler.recoverInline(self)
                else:
                    self._errHandler.reportMatch(self)
                    self.consume()
                self.state = 32
                self.match(catlParser.T__4)
                self.state = 33
                localctx.child = self.catlProperty(5)
                pass
            else:
                raise NoViableAltException(self)

            self._ctx.stop = self._input.LT(-1)
            self.state = 55
            self._errHandler.sync(self)
            _alt = self._interp.adaptivePredict(self._input,2,self._ctx)
            while _alt!=2 and _alt!=ATN.INVALID_ALT_NUMBER:
                if _alt==1:
                    if self._parseListeners is not None:
                        self.triggerExitRuleEvent()
                    _prevctx = localctx
                    self.state = 53
                    self._errHandler.sync(self)
                    la_ = self._interp.adaptivePredict(self._input,1,self._ctx)
                    if la_ == 1:
                        localctx = catlParser.FormulaContext(self, catlParser.CatlPropertyContext(self, _parentctx, _parentState))
                        localctx.left = _prevctx
                        self.pushNewRecursionContext(localctx, _startState, self.RULE_catlProperty)
                        self.state = 36
                        if not self.precpred(self._ctx, 4):
                            from antlr4.error.Errors import FailedPredicateException
                            raise FailedPredicateException(self, "self.precpred(self._ctx, 4)")
                        self.state = 37
                        localctx.op = self.match(catlParser.IMPLIES)
                        self.state = 38
                        localctx.right = self.catlProperty(5)
                        pass

                    elif la_ == 2:
                        localctx = catlParser.FormulaContext(self, catlParser.CatlPropertyContext(self, _parentctx, _parentState))
                        localctx.left = _prevctx
                        self.pushNewRecursionContext(localctx, _startState, self.RULE_catlProperty)
                        self.state = 39
                        if not self.precpred(self._ctx, 3):
                            from antlr4.error.Errors import FailedPredicateException
                            raise FailedPredicateException(self, "self.precpred(self._ctx, 3)")
                        self.state = 40
                        localctx.op = self.match(catlParser.AND)
                        self.state = 41
                        localctx.right = self.catlProperty(4)
                        pass

                    elif la_ == 3:
                        localctx = catlParser.FormulaContext(self, catlParser.CatlPropertyContext(self, _parentctx, _parentState))
                        localctx.left = _prevctx
                        self.pushNewRecursionContext(localctx, _startState, self.RULE_catlProperty)
                        self.state = 42
                        if not self.precpred(self._ctx, 2):
                            from antlr4.error.Errors import FailedPredicateException
                            raise FailedPredicateException(self, "self.precpred(self._ctx, 2)")
                        self.state = 43
                        localctx.op = self.match(catlParser.OR)
                        self.state = 44
                        localctx.right = self.catlProperty(3)
                        pass

                    elif la_ == 4:
                        localctx = catlParser.FormulaContext(self, catlParser.CatlPropertyContext(self, _parentctx, _parentState))
                        localctx.left = _prevctx
                        self.pushNewRecursionContext(localctx, _startState, self.RULE_catlProperty)
                        self.state = 45
                        if not self.precpred(self._ctx, 1):
                            from antlr4.error.Errors import FailedPredicateException
                            raise FailedPredicateException(self, "self.precpred(self._ctx, 1)")
                        self.state = 46
                        localctx.op = self.match(catlParser.UNTIL)
                        self.state = 47
                        self.match(catlParser.T__2)
                        self.state = 48
                        localctx.low = self._input.LT(1)
                        _la = self._input.LA(1)
                        if not(_la==18 or _la==19):
                            localctx.low = self._errHandler.recoverInline(self)
                        else:
                            self._errHandler.reportMatch(self)
                            self.consume()
                        self.state = 49
                        self.match(catlParser.T__3)
                        self.state = 50
                        localctx.high = self._input.LT(1)
                        _la = self._input.LA(1)
                        if not(_la==18 or _la==19):
                            localctx.high = self._errHandler.recoverInline(self)
                        else:
                            self._errHandler.reportMatch(self)
                            self.consume()
                        self.state = 51
                        self.match(catlParser.T__4)
                        self.state = 52
                        localctx.right = self.catlProperty(2)
                        pass

             
                self.state = 57
                self._errHandler.sync(self)
                _alt = self._interp.adaptivePredict(self._input,2,self._ctx)

        except RecognitionException as re:
            localctx.exception = re
            self._errHandler.reportError(self, re)
            self._errHandler.recover(self, re)
        finally:
            self.unrollRecursionContexts(_parentctx)
        return localctx


    class PredicateContext(ParserRuleContext):
        __slots__ = 'parser'

        def __init__(self, parser, parent:ParserRuleContext=None, invokingState:int=-1):
            super().__init__(parent, invokingState)
            self.parser = parser
            self.op = None # Token
            self.duration = None # Token
            self.proposition = None # Token

        def capabilities(self):
            return self.getTypedRuleContext(catlParser.CapabilitiesContext,0)


        def VARIABLE(self):
            return self.getToken(catlParser.VARIABLE, 0)

        def RATIONAL(self):
            return self.getToken(catlParser.RATIONAL, 0)

        def INT(self):
            return self.getToken(catlParser.INT, 0)

        def resources(self):
            return self.getTypedRuleContext(catlParser.ResourcesContext,0)


        def BOOLEAN(self):
            return self.getToken(catlParser.BOOLEAN, 0)

        def getRuleIndex(self):
            return catlParser.RULE_predicate

        def enterRule(self, listener:ParseTreeListener):
            if hasattr( listener, "enterPredicate" ):
                listener.enterPredicate(self)

        def exitRule(self, listener:ParseTreeListener):
            if hasattr( listener, "exitPredicate" ):
                listener.exitPredicate(self)

        def accept(self, visitor:ParseTreeVisitor):
            if hasattr( visitor, "visitPredicate" ):
                return visitor.visitPredicate(self)
            else:
                return visitor.visitChildren(self)




    def predicate(self):

        localctx = catlParser.PredicateContext(self, self._ctx, self.state)
        self.enterRule(localctx, 2, self.RULE_predicate)
        self._la = 0 # Token type
        try:
            self.state = 72
            self._errHandler.sync(self)
            token = self._input.LA(1)
            if token in [6]:
                self.enterOuterAlt(localctx, 1)
                self.state = 58
                localctx.op = self.match(catlParser.T__5)
                self.state = 59
                self.match(catlParser.T__0)
                self.state = 60
                localctx.duration = self._input.LT(1)
                _la = self._input.LA(1)
                if not(_la==18 or _la==19):
                    localctx.duration = self._errHandler.recoverInline(self)
                else:
                    self._errHandler.reportMatch(self)
                    self.consume()
                self.state = 61
                self.match(catlParser.T__3)
                self.state = 62
                localctx.proposition = self.match(catlParser.VARIABLE)
                self.state = 63
                self.match(catlParser.T__3)
                self.state = 64
                self.capabilities()
                self.state = 67
                self._errHandler.sync(self)
                _la = self._input.LA(1)
                if _la==4:
                    self.state = 65
                    self.match(catlParser.T__3)
                    self.state = 66
                    self.resources()


                self.state = 69
                self.match(catlParser.T__1)
                pass
            elif token in [16]:
                self.enterOuterAlt(localctx, 2)
                self.state = 71
                localctx.op = self.match(catlParser.BOOLEAN)
                pass
            else:
                raise NoViableAltException(self)

        except RecognitionException as re:
            localctx.exception = re
            self._errHandler.reportError(self, re)
            self._errHandler.recover(self, re)
        finally:
            self.exitRule()
        return localctx


    class CapabilitiesContext(ParserRuleContext):
        __slots__ = 'parser'

        def __init__(self, parser, parent:ParserRuleContext=None, invokingState:int=-1):
            super().__init__(parent, invokingState)
            self.parser = parser

        def capabilityRequest(self, i:int=None):
            if i is None:
                return self.getTypedRuleContexts(catlParser.CapabilityRequestContext)
            else:
                return self.getTypedRuleContext(catlParser.CapabilityRequestContext,i)


        def getRuleIndex(self):
            return catlParser.RULE_capabilities

        def enterRule(self, listener:ParseTreeListener):
            if hasattr( listener, "enterCapabilities" ):
                listener.enterCapabilities(self)

        def exitRule(self, listener:ParseTreeListener):
            if hasattr( listener, "exitCapabilities" ):
                listener.exitCapabilities(self)

        def accept(self, visitor:ParseTreeVisitor):
            if hasattr( visitor, "visitCapabilities" ):
                return visitor.visitCapabilities(self)
            else:
                return visitor.visitChildren(self)




    def capabilities(self):

        localctx = catlParser.CapabilitiesContext(self, self._ctx, self.state)
        self.enterRule(localctx, 4, self.RULE_capabilities)
        self._la = 0 # Token type
        try:
            self.enterOuterAlt(localctx, 1)
            self.state = 74
            self.match(catlParser.T__6)
            self.state = 75
            self.capabilityRequest()
            self.state = 80
            self._errHandler.sync(self)
            _la = self._input.LA(1)
            while _la==4:
                self.state = 76
                self.match(catlParser.T__3)
                self.state = 77
                self.capabilityRequest()
                self.state = 82
                self._errHandler.sync(self)
                _la = self._input.LA(1)

            self.state = 83
            self.match(catlParser.T__7)
        except RecognitionException as re:
            localctx.exception = re
            self._errHandler.reportError(self, re)
            self._errHandler.recover(self, re)
        finally:
            self.exitRule()
        return localctx


    class CapabilityRequestContext(ParserRuleContext):
        __slots__ = 'parser'

        def __init__(self, parser, parent:ParserRuleContext=None, invokingState:int=-1):
            super().__init__(parent, invokingState)
            self.parser = parser
            self.cap = None # Token
            self.count = None # Token

        def VARIABLE(self):
            return self.getToken(catlParser.VARIABLE, 0)

        def INT(self):
            return self.getToken(catlParser.INT, 0)

        def getRuleIndex(self):
            return catlParser.RULE_capabilityRequest

        def enterRule(self, listener:ParseTreeListener):
            if hasattr( listener, "enterCapabilityRequest" ):
                listener.enterCapabilityRequest(self)

        def exitRule(self, listener:ParseTreeListener):
            if hasattr( listener, "exitCapabilityRequest" ):
                listener.exitCapabilityRequest(self)

        def accept(self, visitor:ParseTreeVisitor):
            if hasattr( visitor, "visitCapabilityRequest" ):
                return visitor.visitCapabilityRequest(self)
            else:
                return visitor.visitChildren(self)




    def capabilityRequest(self):

        localctx = catlParser.CapabilityRequestContext(self, self._ctx, self.state)
        self.enterRule(localctx, 6, self.RULE_capabilityRequest)
        try:
            self.enterOuterAlt(localctx, 1)
            self.state = 85
            self.match(catlParser.T__0)
            self.state = 86
            localctx.cap = self.match(catlParser.VARIABLE)
            self.state = 87
            self.match(catlParser.T__3)
            self.state = 88
            localctx.count = self.match(catlParser.INT)
            self.state = 89
            self.match(catlParser.T__1)
        except RecognitionException as re:
            localctx.exception = re
            self._errHandler.reportError(self, re)
            self._errHandler.recover(self, re)
        finally:
            self.exitRule()
        return localctx


    class ResourcesContext(ParserRuleContext):
        __slots__ = 'parser'

        def __init__(self, parser, parent:ParserRuleContext=None, invokingState:int=-1):
            super().__init__(parent, invokingState)
            self.parser = parser

        def resourceRequest(self, i:int=None):
            if i is None:
                return self.getTypedRuleContexts(catlParser.ResourceRequestContext)
            else:
                return self.getTypedRuleContext(catlParser.ResourceRequestContext,i)


        def getRuleIndex(self):
            return catlParser.RULE_resources

        def enterRule(self, listener:ParseTreeListener):
            if hasattr( listener, "enterResources" ):
                listener.enterResources(self)

        def exitRule(self, listener:ParseTreeListener):
            if hasattr( listener, "exitResources" ):
                listener.exitResources(self)

        def accept(self, visitor:ParseTreeVisitor):
            if hasattr( visitor, "visitResources" ):
                return visitor.visitResources(self)
            else:
                return visitor.visitChildren(self)




    def resources(self):

        localctx = catlParser.ResourcesContext(self, self._ctx, self.state)
        self.enterRule(localctx, 8, self.RULE_resources)
        self._la = 0 # Token type
        try:
            self.enterOuterAlt(localctx, 1)
            self.state = 91
            self.match(catlParser.T__6)
            self.state = 92
            self.resourceRequest()
            self.state = 97
            self._errHandler.sync(self)
            _la = self._input.LA(1)
            while _la==4:
                self.state = 93
                self.match(catlParser.T__3)
                self.state = 94
                self.resourceRequest()
                self.state = 99
                self._errHandler.sync(self)
                _la = self._input.LA(1)

            self.state = 100
            self.match(catlParser.T__7)
        except RecognitionException as re:
            localctx.exception = re
            self._errHandler.reportError(self, re)
            self._errHandler.recover(self, re)
        finally:
            self.exitRule()
        return localctx


    class ResourceRequestContext(ParserRuleContext):
        __slots__ = 'parser'

        def __init__(self, parser, parent:ParserRuleContext=None, invokingState:int=-1):
            super().__init__(parent, invokingState)
            self.parser = parser
            self.res = None # Token
            self.quantity = None # Token

        def VARIABLE(self):
            return self.getToken(catlParser.VARIABLE, 0)

        def RATIONAL(self):
            return self.getToken(catlParser.RATIONAL, 0)

        def INT(self):
            return self.getToken(catlParser.INT, 0)

        def getRuleIndex(self):
            return catlParser.RULE_resourceRequest

        def enterRule(self, listener:ParseTreeListener):
            if hasattr( listener, "enterResourceRequest" ):
                listener.enterResourceRequest(self)

        def exitRule(self, listener:ParseTreeListener):
            if hasattr( listener, "exitResourceRequest" ):
                listener.exitResourceRequest(self)

        def accept(self, visitor:ParseTreeVisitor):
            if hasattr( visitor, "visitResourceRequest" ):
                return visitor.visitResourceRequest(self)
            else:
                return visitor.visitChildren(self)




    def resourceRequest(self):

        localctx = catlParser.ResourceRequestContext(self, self._ctx, self.state)
        self.enterRule(localctx, 10, self.RULE_resourceRequest)
        self._la = 0 # Token type
        try:
            self.enterOuterAlt(localctx, 1)
            self.state = 102
            self.match(catlParser.T__0)
            self.state = 103
            localctx.res = self.match(catlParser.VARIABLE)
            self.state = 104
            self.match(catlParser.T__3)
            self.state = 105
            localctx.quantity = self._input.LT(1)
            _la = self._input.LA(1)
            if not(_la==18 or _la==19):
                localctx.quantity = self._errHandler.recoverInline(self)
            else:
                self._errHandler.reportMatch(self)
                self.consume()
            self.state = 106
            self.match(catlParser.T__1)
        except RecognitionException as re:
            localctx.exception = re
            self._errHandler.reportError(self, re)
            self._errHandler.recover(self, re)
        finally:
            self.exitRule()
        return localctx



    def sempred(self, localctx:RuleContext, ruleIndex:int, predIndex:int):
        if self._predicates == None:
            self._predicates = dict()
        self._predicates[0] = self.catlProperty_sempred
        pred = self._predicates.get(ruleIndex, None)
        if pred is None:
            raise Exception("No predicate with index:" + str(ruleIndex))
        else:
            return pred(localctx, predIndex)

    def catlProperty_sempred(self, localctx:CatlPropertyContext, predIndex:int):
            if predIndex == 0:
                return self.precpred(self._ctx, 4)
         

            if predIndex == 1:
                return self.precpred(self._ctx, 3)
         

            if predIndex == 2:
                return self.precpred(self._ctx, 2)
         

            if predIndex == 3:
                return self.precpred(self._ctx, 1)
         





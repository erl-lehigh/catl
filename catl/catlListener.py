# Generated from catl.g4 by ANTLR 4.8
from antlr4 import *
if __name__ is not None and "." in __name__:
    from .catlParser import catlParser
else:
    from catlParser import catlParser

'''
 Copyright (C) 2018-2020 Cristian Ioan Vasile <cvasile@lehigh.edu>
 Explainable Robotics Lab (ERL), Autonomous and Intelligent Robotics (AIR) Lab,
 Lehigh University
 Hybrid and Networked Systems (HyNeSs) Group, BU Robotics Lab, Boston University
 See license.txt file for license information.
'''


# This class defines a complete listener for a parse tree produced by catlParser.
class catlListener(ParseTreeListener):

    # Enter a parse tree produced by catlParser#catlPredicate.
    def enterCatlPredicate(self, ctx:catlParser.CatlPredicateContext):
        pass

    # Exit a parse tree produced by catlParser#catlPredicate.
    def exitCatlPredicate(self, ctx:catlParser.CatlPredicateContext):
        pass


    # Enter a parse tree produced by catlParser#catlLimit.
    def enterCatlLimit(self, ctx:catlParser.CatlLimitContext):
        pass

    # Exit a parse tree produced by catlParser#catlLimit.
    def exitCatlLimit(self, ctx:catlParser.CatlLimitContext):
        pass


    # Enter a parse tree produced by catlParser#formula.
    def enterFormula(self, ctx:catlParser.FormulaContext):
        pass

    # Exit a parse tree produced by catlParser#formula.
    def exitFormula(self, ctx:catlParser.FormulaContext):
        pass


    # Enter a parse tree produced by catlParser#parprop.
    def enterParprop(self, ctx:catlParser.ParpropContext):
        pass

    # Exit a parse tree produced by catlParser#parprop.
    def exitParprop(self, ctx:catlParser.ParpropContext):
        pass


    # Enter a parse tree produced by catlParser#predicate.
    def enterPredicate(self, ctx:catlParser.PredicateContext):
        pass

    # Exit a parse tree produced by catlParser#predicate.
    def exitPredicate(self, ctx:catlParser.PredicateContext):
        pass


    # Enter a parse tree produced by catlParser#limit.
    def enterLimit(self, ctx:catlParser.LimitContext):
        pass

    # Exit a parse tree produced by catlParser#limit.
    def exitLimit(self, ctx:catlParser.LimitContext):
        pass


    # Enter a parse tree produced by catlParser#capabilities.
    def enterCapabilities(self, ctx:catlParser.CapabilitiesContext):
        pass

    # Exit a parse tree produced by catlParser#capabilities.
    def exitCapabilities(self, ctx:catlParser.CapabilitiesContext):
        pass


    # Enter a parse tree produced by catlParser#capabilityRequest.
    def enterCapabilityRequest(self, ctx:catlParser.CapabilityRequestContext):
        pass

    # Exit a parse tree produced by catlParser#capabilityRequest.
    def exitCapabilityRequest(self, ctx:catlParser.CapabilityRequestContext):
        pass


    # Enter a parse tree produced by catlParser#resources.
    def enterResources(self, ctx:catlParser.ResourcesContext):
        pass

    # Exit a parse tree produced by catlParser#resources.
    def exitResources(self, ctx:catlParser.ResourcesContext):
        pass


    # Enter a parse tree produced by catlParser#resourceRequest.
    def enterResourceRequest(self, ctx:catlParser.ResourceRequestContext):
        pass

    # Exit a parse tree produced by catlParser#resourceRequest.
    def exitResourceRequest(self, ctx:catlParser.ResourceRequestContext):
        pass



del catlParser
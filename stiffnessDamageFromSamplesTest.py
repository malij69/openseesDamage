# -*- coding: utf-8 -*-
"""
Created on Thu Dec 26 22:35:19 2024

@author: malij
"""


# import os as os 
import math
import openseespy.opensees as ops
# import numpy as np
# import opsvis as opsv
# import matplotlib.pyplot as plt
import pickle


#turn this parameter on as 'yes' to capture stiffness and period of intact frame
# captureIntactParameters = 'yes'
captureIntactParameters = 'no'
# testingParameter = 'yes'
testingParameter = 'no'

with open('sampless.pkl', 'rb') as f:
# with open('samplessTest_21.pkl', 'rb') as f:
    samples = pickle.load(f)

stiffnessMatrixToatal = []
elementStatesTotal = []
periodsRaw = []
periodsLoaded = []
elementForcesTotal = []

# samples = [[1],[2]]
numbering = 1
for combinations in samples:
    # combinations lists all selected samples for each spicific number of damage members.
    # for example a list of 8 if 2 members are damaged or 20 if 3 members are damaged , etc
    for combination in combinations:
        # (combination[0]) === the list of location       of  damaged members in samples
        # (combination[1]) === the list of quantification of  damaged members in samples
        # combination is the list of selected damage locations and quantifications
        # print (len(combination[0]))
        # print (len(combination[1]))

        # Basic units
        M = 1.0           # Output units: meters
        N = 1.0           # Output units: newtons
        Sec = 1.0         # Output units: seconds

        # Parameters
        leanCol = 'yes'
        numberOfStories = 3  # Define this variable
        LCol = 3.0  # Define this variable
        numberOfBays = 3 # Define this variable
        LBeam = 5.0  # Define this variable


        LBeam = 5.0 * M      # Beam length
        LCol = 3.0 * M       # Column height
        hCol1 = 0.5 * M     # Type 1 beam and column height

        # Material properties
        fpc = -25e6
        epsc0 = -0.0024
        Ec = 2 * fpc / epsc0  # Concrete elastic modulus
        Es = 2.2e11           # Steel elastic modulus
        

        # Define units
        cm = M / 100.0        # Centimeter in meters
        mm = M / 1000.0       # Millimeter in meters
        kN = N / 1000.0       # Kilonewton
        pascal = N / (M ** 2) # Pascal
        mpa = pascal / 1e6    # Megapascal
        PI = 2 * 3.141592653589793 / 2  # Pi constant
        g = 9.806 * M / (Sec ** 2)      # Gravitational acceleration
        kg = g * N                      # Kilogram
        rho = kg / (M ** 3)             # Density (kg/m³)

        # Bar radius conversion
        # barRadius = 20  # Example radius in mm (adjust as needed)
        barRadius = math.sqrt((hCol1 * hCol1 * 0.04) / (3.14 * 8)) * 1000  # mm
        barRadius = int(math.ceil(barRadius))
        print(f"barRadius == {barRadius}")
        
        Type1BarradiusCol = barRadius * mm

        # Number of bars in each corner
        upReinNum = 4
        middeliUpReinNum = 2
        middleDownReinNum = 2
        downReinNum = 4

        # Beam-column type
        typeChangeRate = 1
        numberOfColumnTypes = int(numberOfStories / typeChangeRate)
        numberOfBeamTypes = int(numberOfStories / typeChangeRate)

        # Lists to store column dimensions
        hCol = [hCol1]
        bCol = [hCol1]
        cc = hCol1

        for k in range(2, numberOfColumnTypes + 1):
            h = cc * 0.925
            b = h
            cc = h
            hCol.append(h)
            bCol.append(b)
            locals()[f'hCol{k}'] = h
            locals()[f'bCol{k}'] = b

        # print("h_col:", hCol)

        # Bar parameters for columns
        barradiusCol = [Type1BarradiusCol]
        cc = Type1BarradiusCol * 1000

        for k in range(2, numberOfBeamTypes + 1):
            ba = int(cc * 0.925) / 1000.0
            cc = ba * 1000
            barradiusCol.append(ba)

        # print("bar_radius_col:", barradiusCol)

        # Beam section dimensions
        hBeam1 = hCol1 * 0.90
        bBeam1 = hBeam1
        hBeam = [hBeam1]
        bBeam = [bBeam1]
        cc = hBeam1

        for k in range(2, numberOfBeamTypes + 1):
            h = cc * 0.9
            b = h
            cc = h
            hBeam.append(h)
            bBeam.append(b)
            locals()[f'hBeam{k}'] = h
            locals()[f'bBeam{k}'] = b

        # print("h_beam:", hBeam)

        # Bar parameters for beams
        Type1BarradiusBeam = int(Type1BarradiusCol * 0.80 * 1000) / 1000.0
        BarradiusBeam = [Type1BarradiusBeam]
        cc = Type1BarradiusBeam * 1000

        # print("Type1Bar_radius_beam:", int(cc))

        for k in range(2, numberOfBeamTypes + 1):
            ba = int(cc * 0.9) / 1000.0
            cc = ba * 1000
            BarradiusBeam.append(ba)

        # print("bar_radius_beam:", BarradiusBeam)




        ops.wipe()
        ops.wipeAnalysis()
        ops.model('basic', '-ndm', 2, '-ndf', 3)



        #nodes
        nodeIDs=[]
        leanNodeIDs=[]
        for level in range(1, numberOfStories + 2):
            Y = (level - 1) * LCol
            for pier in range(1, numberOfBays + 2):
                X = (pier - 1) * LBeam
                nodeID = level + pier * 100
                nodeIDs.append(nodeID)
                # actually define node
                ops.node (nodeID, X, Y);
                # print(f"node {nodeID} {X} {Y}")  # This simulates the puts command
                
            #lean col nodes    
            #10000 is the lower spring node and 20000 is for the uper ones  
            if leanCol == 'yes':
                if level == 1:
                    leanPier = numberOfBays + 2
                    leanX = (pier) * LBeam
                    leanNodeID = level + leanPier * 100
                    leanNodeIDs.append(leanNodeID)
                    ops.node (leanNodeID, leanX, Y);
                    # print(f"node {leanNodeID} {leanX} {Y}")

                if 1 < level < numberOfStories + 1:
                    leanPier = numberOfBays + 2
                    leanX = (pier) * LBeam
                    leanNodeID = level + leanPier * 100
                    leanNodeIDs.append(leanNodeID)
                    ops.node (leanNodeID, leanX, Y);
                    ops.node (leanNodeID+ 10000, leanX, Y);
                    ops.node (leanNodeID+ 20000, leanX, Y);
                    # print(f"node {leanNodeID} {leanX} {Y}")
                    # print(f"node {leanNodeID + 10000} {leanX} {Y}")
                    # print(f"node {leanNodeID + 20000} {leanX} {Y}")

                if level == numberOfStories + 1:
                    leanPier = numberOfBays + 2
                    leanX = (pier) * LBeam
                    leanNodeID = level + leanPier * 100
                    leanNodeIDs.append(leanNodeID)
                    ops.node (leanNodeID, leanX, Y);
                    ops.node (leanNodeID+ 10000, leanX, Y);
                    # print(f"node {leanNodeID} {leanX} {Y}")
                    # print(f"node {leanNodeID + 10000} {leanX} {Y}")    
                    
                    
        #boundary conditions

        level = 1
        for pier in range(1, numberOfBays + 2):
            baseNodeID = pier * 100 + level
            # print(f"fix {baseNodeID}  1 1 1;")
            ops.fix(baseNodeID, 1, 1, 1)

        if leanCol == 'yes':
            leanPier = numberOfBays + 2
            leanColBaseNodeID = leanPier * 100 + 1
            ops.fix(leanColBaseNodeID, 1, 1, 0)
            # print(f"fix {leanColBaseNodeID}  1 1 0;")
                    
                
         


        #======calculate the weights, loads and masses=======
        WeightColType = []
        WeightBeamType = []
        QdlBeamType = []
        QdlColType=[]
        hbfb=[]
        hbfc=[]

        for t in range(1, int(numberOfStories / typeChangeRate) + 1):
            
            # hbfb = locals()[f'hCol{t}']
            # hbfb = eval(f'{hbfb}')
            # hbfb_t = hbfb  # h of beam
            hbfb = hCol[t-1]
            hbfc = hbfb
            # hbfc_t = hbfc  # h of column. for now it's equal to that of the beam.
            
            ConcreteGamma = 2450 * rho  # density of Reinforced-Concrete (rho or gama)
            Tslab = 15 * cm  # slab with 15-cm Thickness
            Lslab = 2*LBeam / 2  # assume slabs Length extends a distance of LBeam1/2 in/out of plane
            Qslab = ConcreteGamma * Tslab * Lslab  # longitudinal slab weight
            QBeam = ConcreteGamma * hbfb * hbfb  # Reinforced concrete section weight per length

            QdlBeamType_t = Qslab + QBeam  # dead load distributed along beam.
            QdlBeamType.append(QdlBeamType_t)
            # QdlBeam = eval(f'{QdlBeamType_t}')
            QdlBeam = QdlBeamType[t-1]

            QdlColType_t = ConcreteGamma * hbfc * hbfc  # Reinforced concrete section weight per length
            QdlColType.append(QdlColType_t)
            # QdlCol = eval(f'{QdlColType_t}')
            QdlCol = QdlColType [t-1]

            WeightColType_t = QdlCol * LCol  # total Column weight by newtons
            WeightColType.append(WeightColType_t)
            WeightBeamType_t = QdlBeam * LBeam  # total Beam weight by newtons
            WeightBeamType.append(WeightBeamType_t)
            # print ("WeightColType_t=",WeightColType_t)
            # print ("WeightBeamType_t=",WeightBeamType_t)

            if numberOfStories == typeChangeRate:
                break



        #assign masses
        nodalMasses=[]
        for t in range(1, int(numberOfStories / typeChangeRate) + 2):
            # for level in range((t - 1) * typeChangeRate + 2, max((t - 1) * typeChangeRate + typeChangeRate + 1, numberOfStories + 1)):
            level = (t - 1) * typeChangeRate + 2
            while level <= (t - 1) * typeChangeRate + typeChangeRate + 1 and level <= numberOfStories  + 1:     
                       
                if level == numberOfStories + 1:
                    # print ("level",level)

                    ColWeightFact = 1  # one column in top story
                    # print ("aaaa")
                else:

                    ColWeightFact = 2  # two columns elsewhere
                    # print ("bbbb")
                for pier in range(1, numberOfBays + 2):

                    if pier == 1 or pier == numberOfBays + 1:
                        # print ("cccc")

                        BeamWeightFact = 1  # one beam at exterior nodes
                    else:

                        BeamWeightFact = 2  # two beams elsewhere
                        # print ("dddd")

                    # WeightCol = eval(f'{WeightColType_t}')
                    WeightCol = WeightColType [t-1]
                    # print ("WeightCol==========",WeightCol)
                    # WeightBeam = eval(f'{WeightBeamType_t}')
                    WeightBeam =WeightBeamType[t-1]
                    # print ("WeightBeam==========",WeightBeam)
                    WeightNode = ColWeightFact * WeightCol / 2 + BeamWeightFact * WeightBeam / 2
                    MassNode = WeightNode / g
                    nodeID = level + pier * 100
                    ops.mass(nodeID, MassNode, 1.e-9, 1.e-9)
                    nodalMasses.append(MassNode)
                    # print(f'mass {nodeID} {MassNode} 1.e-9 1.e-9 ;')
                level += 1




        #mp_constraints



        for level in range(2, numberOfStories + 2):
            for pier in range(1, numberOfBays + 1):
                nodeA = level + pier * 100
                nodeB = level + pier * 100 + 1 * 100
                ops.equalDOF(nodeA,nodeB , 2, 3)
                # print(f"equalDOF {nodeA} {nodeB} 2 3")
                
                


        #----material definition-----
         

        # sM is the stiffnessMultipliyer
        def opsMaterials(SM,Ccover,Ccore,Rein):



             fpc = -25.e6
             epsc0 = -0.0024
             Fy = 400e6
             # Ec = 2 * fpc / epsc0  # concrete elastic modulus
             Es = 2.2e11  # steel elastic modulus

             ops.uniaxialMaterial("Concrete01", Ccover, 0.84*fpc*SM, -0.002*SM, -4.8e6*SM, -0.005)
             ops.uniaxialMaterial("Concrete01", Ccore, fpc*SM, epsc0*SM, -5.6e6*SM, -0.015)
             ops.uniaxialMaterial("Steel02", Rein, Fy*SM, Es*SM, 0.01*SM, 18, 0.925, 0.15)

        # stiffnessMultipliyer = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9]
        normalStiffnessMultipliyer = 1
        # damagePhases = [0.9, 0.8, 0.7, 0.6, 0.5, 0.4, 0.3, 0.2, 0.1]
        damagePhases = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9]

        damageSM01 = damagePhases [0]
        damageSM02 = damagePhases [1]
        damageSM03 = damagePhases [2]
        damageSM04 = damagePhases [3]
        damageSM05 = damagePhases [4]
        damageSM06 = damagePhases [5]
        damageSM07 = damagePhases [6]
        damageSM08 = damagePhases [7]
        damageSM09 = damagePhases [8]
        
        #stiffness0.9 means that stiffness is 0.9*intactstiffness for the member


        CcoverNormal , CcoreNormal , ReinNormal  = 2,3,4
        CcoverDamage9, CcoreDamage9, ReinDamage9 = 5,6,7
        CcoverDamage8, CcoreDamage8, ReinDamage8 = 8,9,10
        CcoverDamage7, CcoreDamage7, ReinDamage7 = 11,12,13
        CcoverDamage6, CcoreDamage6, ReinDamage6 = 14,15,16
        CcoverDamage5, CcoreDamage5, ReinDamage5 = 17,18,19
        CcoverDamage4, CcoreDamage4, ReinDamage4 = 20,21,22
        CcoverDamage3, CcoreDamage3, ReinDamage3 = 23,24,25
        CcoverDamage2, CcoreDamage2, ReinDamage2 = 26,27,28
        CcoverDamage1, CcoreDamage1, ReinDamage1 = 29,30,31


         
        opsMaterials(normalStiffnessMultipliyer , CcoverNormal, CcoreNormal, ReinNormal)
        opsMaterials(damageSM09 , CcoverDamage9, CcoreDamage9, ReinDamage9)
        opsMaterials(damageSM08 , CcoverDamage8, CcoreDamage8, ReinDamage8)
        opsMaterials(damageSM07 , CcoverDamage7, CcoreDamage7, ReinDamage7)
        opsMaterials(damageSM06 , CcoverDamage6, CcoreDamage6, ReinDamage6)
        opsMaterials(damageSM05 , CcoverDamage5, CcoreDamage5, ReinDamage5)
        opsMaterials(damageSM04 , CcoverDamage4, CcoreDamage4, ReinDamage4)
        opsMaterials(damageSM03 , CcoverDamage3, CcoreDamage3, ReinDamage3)
        opsMaterials(damageSM02 , CcoverDamage2, CcoreDamage2, ReinDamage2)
        opsMaterials(damageSM01 , CcoverDamage1, CcoreDamage1, ReinDamage1)
        



        #===== sections ============================

        def rec_fiber_section(hbf, bbf, hbfi, bbfi, radius, sectionType, tagOffset,state):
            if state == 'normal': Ccover,Ccore,Rein =  CcoverNormal , CcoreNormal , ReinNormal
            if state == 'stiffness09': Ccore,Ccover,Rein = CcoverDamage9, CcoreDamage9, ReinDamage9
            if state == 'stiffness08': Ccore,Ccover,Rein = CcoverDamage8, CcoreDamage8, ReinDamage8
            if state == 'stiffness07': Ccore,Ccover,Rein = CcoverDamage7, CcoreDamage7, ReinDamage7
            if state == 'stiffness06': Ccore,Ccover,Rein = CcoverDamage6, CcoreDamage6, ReinDamage6
            if state == 'stiffness05': Ccore,Ccover,Rein = CcoverDamage5, CcoreDamage5, ReinDamage5
            if state == 'stiffness04': Ccore,Ccover,Rein = CcoverDamage4, CcoreDamage4, ReinDamage4
            if state == 'stiffness03': Ccore,Ccover,Rein = CcoverDamage3, CcoreDamage3, ReinDamage3
            if state == 'stiffness02': Ccore,Ccover,Rein = CcoverDamage2, CcoreDamage2, ReinDamage2
            if state == 'stiffness01': Ccore,Ccover,Rein = CcoverDamage1, CcoreDamage1, ReinDamage1

            # Define section with fibers
            ops.section('Fiber', sectionType+tagOffset)
            # print('fiber', sectionType+tagOffset )

            # Core concrete (Ccore)
            ops.patch('quad', Ccore, 3, 3, 
                      -hbfi / 2,  bbfi / 2, 
                      -hbfi / 2, -bbfi / 2, 
                        hbfi / 2, -bbfi / 2, 
                        hbfi / 2,  bbfi / 2)

            # Cover concrete (Ccover)
            ops.patch('quad', Ccover, 1, 3,
                      -hbf / 2,  bbf / 2, 
                      -hbf / 2, -bbf / 2, 
                      -hbfi / 2, -bbf / 2, 
                      -hbfi / 2,  bbf / 2)

            ops.patch('quad', Ccover, 1, 3,
                      -hbfi / 2, -bbfi / 2, 
                      -hbfi / 2, -bbf / 2, 
                        hbfi / 2, -bbf / 2, 
                        hbfi / 2, -bbfi / 2)

            ops.patch('quad', Ccover, 3, 1,
                        hbfi / 2,  bbf / 2, 
                        hbfi / 2, -bbf / 2, 
                        hbf / 2, -bbf / 2, 
                        hbf / 2,  bbf / 2)

            ops.patch('quad', Ccover, 1, 3,
                      -hbfi / 2,  bbf / 2, 
                      -hbfi / 2,  bbfi / 2, 
                        hbfi / 2,  bbfi / 2, 
                        hbfi / 2,  bbf / 2)
            # Steel reinforcement layers
            area = math.pi * radius**2  # Cross-sectional area of rebar

            ops.layer('straight', Rein, upReinNum, area,
                        hbfi / 2,  bbfi / 2,
                        hbfi / 2, -bbfi / 2)

            ops.layer('straight', Rein, middeliUpReinNum, area,
                        hbfi / 4,  bbfi / 2,
                        hbfi / 4, -bbfi / 2)

            ops.layer('straight', Rein, middleDownReinNum, area,
                      -hbfi / 4,  bbfi / 2,
                      -hbfi / 4, -bbfi / 2)

            ops.layer('straight', Rein, downReinNum, area,
                      -hbfi / 2,  bbfi / 2,
                      -hbfi / 2, -bbfi / 2)

        # create sections:

        tagOffsetColNormal = 0            
        tagOffsetBeamNormal = 100

        tagOffsetColDamage1 = tagOffsetColNormal+200            
        tagOffsetBeamDamage1 = tagOffsetBeamNormal+200

        tagOffsetColDamage2 = tagOffsetColNormal+400            
        tagOffsetBeamDamage2 = tagOffsetBeamNormal+400    
        
        tagOffsetColDamage3 = tagOffsetColNormal+600            
        tagOffsetBeamDamage3 = tagOffsetBeamNormal+600

        tagOffsetColDamage4 = tagOffsetColNormal+800            
        tagOffsetBeamDamage4 = tagOffsetBeamNormal+800   
        
        tagOffsetColDamage5 = tagOffsetColNormal+1000            
        tagOffsetBeamDamage5 = tagOffsetBeamNormal+1000

        tagOffsetColDamage6 = tagOffsetColNormal+1200           
        tagOffsetBeamDamage6 = tagOffsetBeamNormal+1200
        
        tagOffsetColDamage7 = tagOffsetColNormal+1400            
        tagOffsetBeamDamage7 = tagOffsetBeamNormal+1400   
        
        tagOffsetColDamage8 = tagOffsetColNormal+1600            
        tagOffsetBeamDamage8 = tagOffsetBeamNormal+1600

        tagOffsetColDamage9 = tagOffsetColNormal+1800            
        tagOffsetBeamDamage9 = tagOffsetBeamNormal+1800  


        colType=1
        for x in hCol:
            sectionType = colType
            radius = barradiusCol[colType - 1]
            hbf = x
            hbfi = hbf - 2 * 0.05
            bbf = x
            bbfi = bbf - 2 * 0.05
            rec_fiber_section (hbf, bbf, hbfi, bbfi, radius, sectionType, tagOffsetColNormal, 'normal')
            rec_fiber_section (hbf, bbf, hbfi, bbfi, radius, sectionType, tagOffsetColDamage9, 'stiffness09')
            rec_fiber_section (hbf, bbf, hbfi, bbfi, radius, sectionType, tagOffsetColDamage8, 'stiffness08')
            rec_fiber_section (hbf, bbf, hbfi, bbfi, radius, sectionType, tagOffsetColDamage7, 'stiffness07')
            rec_fiber_section (hbf, bbf, hbfi, bbfi, radius, sectionType, tagOffsetColDamage6, 'stiffness06')
            rec_fiber_section (hbf, bbf, hbfi, bbfi, radius, sectionType, tagOffsetColDamage5, 'stiffness05')
            rec_fiber_section (hbf, bbf, hbfi, bbfi, radius, sectionType, tagOffsetColDamage4, 'stiffness04')
            rec_fiber_section (hbf, bbf, hbfi, bbfi, radius, sectionType, tagOffsetColDamage3, 'stiffness03')
            rec_fiber_section (hbf, bbf, hbfi, bbfi, radius, sectionType, tagOffsetColDamage2, 'stiffness02')
            rec_fiber_section (hbf, bbf, hbfi, bbfi, radius, sectionType, tagOffsetColDamage1, 'stiffness01')
            colType += 1
            
         

            
        beamType = 1
        for x in hBeam:
            sectionType = beamType
            radius = BarradiusBeam[beamType - 1]
            y = bBeam[beamType - 1]
            hbf = x
            hbfi = hbf - 2 * 0.05
            bbf = y
            bbfi = bbf - 2 * 0.05
            
            rec_fiber_section (hbf, bbf, hbfi, bbfi, radius, sectionType, tagOffsetBeamNormal, 'normal' )
            rec_fiber_section (hbf, bbf, hbfi, bbfi, radius, sectionType, tagOffsetBeamDamage9, 'stiffness09')
            rec_fiber_section (hbf, bbf, hbfi, bbfi, radius, sectionType, tagOffsetBeamDamage8, 'stiffness08')
            rec_fiber_section (hbf, bbf, hbfi, bbfi, radius, sectionType, tagOffsetBeamDamage7, 'stiffness07')
            rec_fiber_section (hbf, bbf, hbfi, bbfi, radius, sectionType, tagOffsetBeamDamage6, 'stiffness06')
            rec_fiber_section (hbf, bbf, hbfi, bbfi, radius, sectionType, tagOffsetBeamDamage5, 'stiffness05')
            rec_fiber_section (hbf, bbf, hbfi, bbfi, radius, sectionType, tagOffsetBeamDamage4, 'stiffness04')
            rec_fiber_section (hbf, bbf, hbfi, bbfi, radius, sectionType, tagOffsetBeamDamage3, 'stiffness03')
            rec_fiber_section (hbf, bbf, hbfi, bbfi, radius, sectionType, tagOffsetBeamDamage2, 'stiffness02')
            rec_fiber_section (hbf, bbf, hbfi, bbfi, radius, sectionType, tagOffsetBeamDamage1, 'stiffness01')
            beamType += 1
            
                    

                    

                    
          

        # Set up geometric transformations for elements
        IDColTransf = 1  # All columns
        IDBeamTransf = 2  # All beams
        ColTransfType = "PDelta"  # Options: Linear, PDelta, Corotational

        ops.geomTransf(ColTransfType, IDColTransf)  # Columns
        ops.geomTransf("PDelta", IDBeamTransf)  # Beams




        #lening wall materials and parameters
        Es = 2e11
        TrussMatID = 600  # Material ID
        Arigid = 10000.0  # Area of truss section (much larger than A of frame elements)
        Irigid = 1000000.0  # Moment of inertia for P-Delta columns
        np=5
        ops.uniaxialMaterial('Elastic',TrussMatID,Es)
        # print(f"uniaxialMaterial Elastic {TrussMatID} {Es}")




        #=== elements ======================
        
        # element assignment for Columns
        columnType = 1
        N0col = 1000  # Column element numbers
        

        damageLocation = 0
        damagePhase = 0.1
        colElements = []
        elementsStates = []
        damagedElements = combination [0]
        # print ('damagedElements==',damagedElements)
        if captureIntactParameters == 'yes': damagedElements = [0,0]

        
        print (damagedElements)
        for level in range(1, numberOfStories + 1):
            for pier in range(1, numberOfBays + 2):
                elemID = N0col + level + pier * 100
                nodeI = level + pier * 100
                nodeJ = (level + 1) + pier * 100
                colElements.append(elemID)
                # print ("aaaaaaaaaaaaaaa=",elemID, nodeI, nodeJ, np, columnType, IDColTransf)
                if elemID in damagedElements:
                    index = damagedElements.index(elemID)
                    damagePhase = combination [1] [index]
                    # print (elemID)
                    # print (damagePhase)
                    for num in range (0,len(damagePhases)):
                        # print (num,'num',num)
                        if damagePhase == damagePhases[num]:
                            offset = (num+1) *200
                            ops.element('nonlinearBeamColumn', elemID, nodeI, nodeJ, np, columnType+offset, IDColTransf)
                            elementsStates.append(damagePhase)  
                      

                else:
                    ops.element('nonlinearBeamColumn', elemID, nodeI, nodeJ, np, columnType, IDColTransf)
                    elementsStates.append(1)
                    
                
                
            if typeChangeRate == 1:
                columnType += 1
            else:
                if (level / typeChangeRate) == int((level + 1) / typeChangeRate):
                    columnType += 1

            if leanCol == 'yes':
                leanPier = numberOfBays + 2
                if level == 1:
                    leanElemID = N0col + level + leanPier * 100
                    leanNodeI = level + leanPier * 100
                    leanNodeJ = (level + 1) + leanPier * 100 + 10000
                    # print ("aaaaaaaaaaaaaaa=",leanElemID, leanNodeI, leanNodeJ, Arigid, Es, Irigid, IDColTransf)

                    ops.element('elasticBeamColumn', leanElemID, leanNodeI, leanNodeJ, Arigid, Es, Irigid, IDColTransf)
                elif level > 1:
                    leanElemID = N0col + level + leanPier * 100
                    leanNodeI = level + leanPier * 100 + 20000
                    leanNodeJ = (level + 1) + leanPier * 100 + 10000
                    # print ("aaaaaaaaaaaaaaa=",leanElemID, leanNodeI, leanNodeJ, Arigid, Es, Irigid, IDColTransf)

                    ops.element('elasticBeamColumn', leanElemID, leanNodeI, leanNodeJ, Arigid, Es, Irigid, IDColTransf)


        # element assignment for  Beams
       
        beamType = 101
        N0beam = 2000  # Beam element numbers
        M0 = 0
        beamElements = []
        for level in range(2, numberOfStories + 2):
            for bay in range(1, numberOfBays + 1):
                elemID = N0beam + level + bay * 100
                nodeI = M0 + level + bay * 100
                nodeJ = M0 + level + (bay + 1) * 100
                beamElements.append(elemID)
                # print ("aaaaaaaaaaaaaaa=",elemID, nodeI, nodeJ, np, beamType, IDBeamTransf)
                if elemID in damagedElements:
                    index = damagedElements.index(elemID)
                    damagePhase = combination [1] [index]
                    # print (elemID)
                    # print (damagePhase)
                    for num in range (0,len(damagePhases)):
                        if damagePhase == damagePhases[num]:
                            offset = (num+1) *200
                            ops.element('nonlinearBeamColumn', elemID, nodeI, nodeJ, np, beamType+offset, IDColTransf)
                            elementsStates.append(damagePhase)
                else:
                    ops.element('nonlinearBeamColumn', elemID, nodeI, nodeJ, np, beamType, IDBeamTransf)
                    elementsStates.append(1)
            if typeChangeRate == 1:
                beamType += 1
            else:
                if ((level - 1) / typeChangeRate) == int((level + 1) / typeChangeRate):
                    beamType += 1        
                

        # ======= assemble member elements ==================
        allElementsList = colElements+beamElements
        # print ('allelementlistallelementlist',allElementsList)
                
        # ====== Leaning Wall Links ======

        for level in range(2, numberOfStories + 2):
            leanBay = numberOfBays + 1
            leanElemID = N0beam + level + leanBay * 100
            leanNodeI = M0 + level + leanBay * 100
            leanNodeJ = M0 + level + (leanBay + 1) * 100
            ops.element('truss',leanElemID,leanNodeI,leanNodeJ,Arigid,TrussMatID)
            # print(f"element truss {leanElemID} {leanNodeI} {leanNodeJ} {Arigid} {TrussMatID}")

        # Function for rotational leaning columns
        def rotLeaningCol(eleID, nodeR, nodeC):
            K = 1e-9  # Spring stiffness
            
            # Create the material and zero-length element (spring)
            ops.uniaxialMaterial('Elastic', eleID, K)
            ops.element('zeroLength', eleID, nodeR, nodeC, '-mat', eleID, '-dir', 6)
            
            # Constrain the translational DOF with a multi-point constraint
            # retained node, constrained node, DOFs
            ops.equalDOF(nodeR, nodeC, 1, 2)

        # Rotational leaning columns setup
        for level in range(2, numberOfStories + 2):
            rotBay = numberOfBays + 1
            rotElemIDA = level + (rotBay + 1) * 100 + 3000
            rotElemIDB = level + (rotBay + 1) * 100 + 14000
            rotNodeI = level + (rotBay + 1) * 100
            rotNodeJDown = level + (rotBay + 1) * 100 + 10000
            rotNodeJUp = level + (rotBay + 1) * 100 + 20000

            if level < (numberOfStories + 1):
                rotLeaningCol(rotElemIDA,rotNodeI,rotNodeJDown)
                rotLeaningCol(rotElemIDB,rotNodeI,rotNodeJUp)
                # print(f"rotLeaningCol {rotElemIDA} {rotNodeI} {rotNodeJDown}")
                # print(f"rotLeaningCol {rotElemIDB} {rotNodeI} {rotNodeJUp}")

            if level == (numberOfStories + 1):
                rotLeaningCol(rotElemIDA,rotNodeI,rotNodeJDown)
                # print(f"rotLeaningCol {rotElemIDA} {rotNodeI} {rotNodeJDown}")
                

        #modal Analysis

        ops.wipeAnalysis()
        eigenvalue = ops.eigen(1)
        T1model = 2.0 * math.pi / math.sqrt(eigenvalue[0])
        print(f"T1model = {T1model} sec")    
                

        # ====== Load Pattern Definition ======
        ops.timeSeries("Linear", 1)
        ops.pattern('Plain', 101, 1)
        type = 1
        for level in range(1, numberOfStories + 1):
            for pier in range(1, numberOfBays + 2):
                elemID = N0col + level + pier * 100
                
                
                # QdlCol = globals()[f"QdlColType{type}"]  # Retrieve the correct QdlColType variable dynamically
                QdlCol = QdlColType[level-1]
                QLL = 0.5 * QdlCol
                Qwind = 0.1 * QdlCol
                QFinal = 0.2 * ((1.2 * QdlCol) + (0.5 * QLL) + (1.5 * Qwind))
                
                Qpdelta = 4 * QFinal  # P-delta load for the leaning column

                if leanCol == "no":
                    ops.eleLoad('-ele', elemID, '-type', '-beamUniform', 0, -(QFinal + Qpdelta))
                
                if leanCol == "yes":
                    ops.eleLoad('-ele', elemID, '-type', '-beamUniform', 0, -QFinal)
                    leanPier = numberOfBays + 2
                    leanNodeID = level + leanPier * 100
                    PDeltaLoad = Qpdelta * numberOfBays * LBeam * LBeam
                    ops.load(leanNodeID, 0.0, -PDeltaLoad, 0.0)
                
                if (level / typeChangeRate) == int((level + 1) / typeChangeRate):
                    type += 1
                    
                    
                    




        # ====== Beams Load Pattern ======
        type = 1
        for level in range(2, numberOfStories + 2):
            for bay in range(1, numberOfBays + 1):
                elemID = N0beam + level + bay * 100
                # QdlBeam = globals()[f"QdlBeamType{type}"]  # Retrieve the correct QdlBeamType variable dynamically
                QdlBeam = QdlBeamType [level-2]
                ops.eleLoad('-ele', elemID, '-type', '-beamUniform', -QdlBeam)
            
            if (level / typeChangeRate) == int((level + 1) / typeChangeRate):
                type += 1

        # ====== Free Node Load ======
        freeNodeID = (numberOfStories + 1) + (numberOfBays + 1) * 100
        ops.pattern('Plain', 1, 1)
        ops.load(freeNodeID, 100.0, 0.0, 0.0)

        # ====== Mode Shape Recorder ======
        for level in range(1, numberOfStories + 1):
            modeShapeNodes = (level + 1) + (numberOfBays + 1) * 100  # Node for mode shapes
            # ops.recorder('Node', '-file', f"modeShapeNodes{level}.out", '-time', '-closeOnWrite', '-node', modeShapeNodes, '-dof', 1, 'eigen', 1)
            


        # Gravity-analysis parameters -- load-controlled static analysis
        ops.wipeAnalysis()
        ops.constraints('Plain')
        ops.numberer('RCM')
        ops.system('BandGeneral')
        ops.test('NormDispIncr', 1.0e-8, 6)
        ops.algorithm('Newton')
        ops.integrator('LoadControl', 0.1)
        ops.analysis('Static')
        ops.analyze(10)
        # print ("here2?")
        # maintain constant gravity loads and reset time to zero
        ops.loadConst('-time', 0.0)
        print("Model Built")
        ops.wipeAnalysis()
        eigenvalue = ops.eigen(1)
        T1modelL = 2.0 * math.pi / math.sqrt(eigenvalue[0])
        print(f"T1bmodel = {T1modelL} sec")    
               
        # opsv.plot_model()
        # plt.show() 

        # ==== compute stiffness matrices ====
        stiffnesMatrices = []
        elementForces = []
        for element in allElementsList:
            numberer = allElementsList.index(element)
            stiffnesMatrices.append(ops.basicStiffness (element))
            
            elementForces.append(ops.basicForce (element))

        stiffnessMatrixToatal.append(stiffnesMatrices)
        elementStatesTotal.append(elementsStates)
        elementForcesTotal.append(elementForces)
        
        periodsRaw.append(T1model)
        periodsLoaded.append(T1modelL)
        #loaded periods could be flawed due to convergence issues 
        print (numbering)
        numbering += 1
        if captureIntactParameters == 'yes':
            stiffnessOfIntactStructure = stiffnesMatrices
            periodRawOfIntactStructure = T1model
            periodLoadedOfIntactStructure = T1modelL
            break
    if captureIntactParameters == 'yes': break


        
            
if captureIntactParameters == 'yes':
    with open('stiffnessOfIntactStructure.pkl', 'wb') as f:
        pickle.dump(stiffnessOfIntactStructure, f)    
    with open('periodRawOfIntactStructure.pkl', 'wb') as f:
        pickle.dump(periodRawOfIntactStructure, f) 
    with open('periodLoadedOfIntactStructure.pkl', 'wb') as f:
        pickle.dump(periodLoadedOfIntactStructure, f) 
    with open('elementForcesTotalIntact.pkl', 'wb') as f:
        pickle.dump(elementForcesTotal, f)         
    
elif testingParameter == 'yes':
    with open('stiffnessMatrixToatalTest.pkl', 'wb') as f:
        pickle.dump(stiffnessMatrixToatal, f)    
    with open('elementStatesTotalTest.pkl', 'wb') as f:
        pickle.dump(elementStatesTotal, f) 
    with open('periodsRawTest.pkl', 'wb') as f:
        pickle.dump(periodsRaw, f) 
    with open('periodsLoadedTest.pkl', 'wb') as f:
        pickle.dump(periodsLoaded, f) 
    with open('elementForcesTotalTest.pkl', 'wb') as f:
        pickle.dump(elementForcesTotal, f) 
        
else: 
    with open('stiffnessMatrixToatal.pkl', 'wb') as f:
        pickle.dump(stiffnessMatrixToatal, f)    
    with open('elementStatesTotal.pkl', 'wb') as f:
        pickle.dump(elementStatesTotal, f) 
    with open('periodsRaw.pkl', 'wb') as f:
        pickle.dump(periodsRaw, f) 
    with open('periodsLoaded.pkl', 'wb') as f:
        pickle.dump(periodsLoaded, f)
    with open('elementForcesTotal.pkl', 'wb') as f:
        pickle.dump(elementForcesTotal, f) 
        



    
    
    
    
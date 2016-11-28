# # include "MSAC.h"
# # include "errorNIETO.h"
# # include "lmmin.h"
#
# #  # ifdef DEBUG_MAP	# if defined, a 2D map will be created (which slows down the process)
#
# using
# namespace
# std
# using
# namespace
# cv

import numpy as np
import cv2
from collections import namedtuple
import random

MODE_LS	= 0
MODE_NIETO =1
INT_MAX = 5
FLT_MAX = 0
RAND_MAX = 5

#This is the data structure passed to the Levenberg-Marquardt procedure */
data_struct = namedtuple("LSS", "Lengths", "midPoints", "K")

        # LSS		# Line segments vectors (3xN)
        # Lengths	# Length of line segments (NxN) (diagonal)
        # midPoints	# Mid points (c=(a+b)/2) (3xN)
        # K		# Camera calibration matrix (3x3)


class MSAC:

    def __init__ (self, mode, imSize, verbose):

        #error mode
        self.mode = mode

        #Arguments
        self.verbose = verbose
        self.width = imSize[1]
        self.height = imSize[0]

        #MSAC parameters
        self.epsilon = 1.e-6
        self.P_inlier = 0.95
        self.T_noise_squared = 0.01623 * 2
        self.min_iters = 5
        self.max_iters = INT_MAX
        self.update_T_iter = False

        #These variables have been initiated temporarily
        self.notify = True
        self.N_I_best = 0 # Number of inliers of the best Consensus Set
        self.J_best = 0. #Cost of the best Consensus Set
        self.MSS = np.zeros(1) # Minimal sample set

        # # Auxiliary variables
        # matrices
        self.a = np.zeros(3)
        self.an = np.zeros(3)
        self.b = np.zeros(3)
        self.bn = np.zeros(3)
        self.li  = np.zeros(3)
        self.c = np.zeros(3)
        self.vp = np.zeros(3)
        self.vpAux = np.zeros(3)
        # # Calibration
        self.K = np.zeros((3,3)) # Approximated Camera calibration matrix

        # # Data(Line Segments)
        #self.Li = np.zeros(3) # Matrix of appended line segments(3xN) for N line segments
        #self.Mi = np.zeros(3)#Matrix of middle points(3 xN)
        #self.Lengths = np.zeros(1)# Matrix of lengths(1xN)

        # Consensus set
        #std::vector < int >
        ##self.CS_idx = np.zeros(1)
        ##self.CS_best = np.zeros(1)#Indexes of line segments: 1 -> belong to CS, 0 -> does not belong
        self.ind_CS_best = np.zeros(1)#Vector of indexes of the Consensus Set double vp_length_ratio

        #Parameters
        self.minimal_sample_set_dimension = 2 # Dimension of the MSS (minimal sample set)

        #Minimal Sample Set
        for i in range(0, self.minimal_sample_set_dimension):
            self.MSS = np.append(self.MSS, [0])

        #(Default) Calibration
        #K = np.array(3,3)
        #K = np.zeros(3,3)
        self.K[0, 0] = self.width
        self.K[0, 2] = self.width/2
        self.K[1, 1] = self.height
        self.K[1, 2] = self.height/2
        self.K[2, 2] = 1

    def fillDataContainers(self, lineSegments):
        numLines = len(lineSegments)
        if (self.verbose):
            print("Line segments: %d\n", numLines)

        # Transform all line segments
        # self.Li =[l_00 l_01 l_02 l_10 l_11 l_12 l_20 l_21 l_22...] where li=[l_i0l_i1l_i2] ^ T is li=an x bn
        self.Li = np.zeros((numLines, 3))
        self.Mi = np.zeros((numLines, 3))
        self.Lengths = np.zeros((numLines, numLines))

        ##self.Lengths.setTo(0)

        ##lineSegments = [[[1,2],[3,4]],[[5,6],[7,8]]]

        # Fill data containers (self.Li, self.Mi, self.Lenghts)
        sum_lengths = 0.

        for i in range(0, numLines):
            #for rho, theta in lineSegments[i]:
            #Extract the end - points
            #print lineSegments[i,0][0]

            rho = lineSegments[i,0][0]
            theta = lineSegments[i,0][1]
            pCons = np.cos(theta)
            pSin = np.sin(theta)
            x0 = pCons * rho
            y0 = pSin * rho
            x1 = int(x0 + 1000 * (-pSin))
            y1 = int(y0 + 1000 * (pCons))
            x2 = int(x0 - 1000 * (-pSin))
            y2 = int(y0 - 1000 * (pCons))

            p1 = [x1,y1]
            p2 = [x2,y2]
            self.a[0] = p1[0]
            self.a[1] = p1[1]
            self.a[2] = 1
            self.b[0] = p2[0]
            self.b[1] = p2[1]
            self.b[2] = 1

            if (self.mode == MODE_NIETO):
                self.c = 0.5 * (self.a+self.b)

            length = np.sqrt((self.b[0]-self.a[0]) * (self.b[0]-self.a[0]) + (self.b[1]-self.a[1]) * (self.b[1]-self.a[1]))
            sum_lengths += length
            self.Lengths[i, i] = length

            if (self.mode == MODE_LS):
                # Normalize into the sphere
                ## Make note of the Inverse
                self.an = np.linalg.inv(self.K) * self.a
                self.bn = np.linalg.inv(self.K) * self.b
            else : # self.mode == MODE_NIETO requires not to calibrate into the sphere
                self.an = self.a
                self.bn = self.b

            testArr = [1, 2, 3]
            # Compute the general form of the line
            self.li = np.cross(self.an, self.bn)
            cv2.normalize(self.li, self.li)

            # Insert line into appended array
            self.Li[i, 0] = self.li[0]
            self.Li[i, 1] = self.li[1]
            self.Li[i, 2] = self.li[2]

            if (self.mode == MODE_NIETO):
                # Store mid-Point too
                self.Mi[i, 0] = self.c[0]
                self.Mi[i, 1] = self.c[1]
                self.Mi[i, 2] = self.c[2]

        self.Lengths = self.Lengths * (1./ sum_lengths)
        return lineSegments

    def GetMinimalSampleSet(self, Li, Lengths, Mi, MSS, vp):
        print len(Li)
        N = len(Li)

        MSS[0] = random.randint(0, RAND_MAX / (N - 1))
        MSS[1] = random.randint(0, RAND_MAX / (N - 1))
        # Generate a pair of samples
        while N <= MSS[0]: #rand() / (RAND_MAX / (N-1)))):
            continue
        while N <= MSS[1] : #rand() / (RAND_MAX / (N-1)))):
            continue

        # Estimate the vanishing point and the residual error
        if (self.mode == MODE_LS):
            tempStore = self.estimateLS(Li, Lengths, MSS, 2, vp)
            Li = tempStore[0]
            Lengths = tempStore[1]
            MSS = tempStore[2]
            vp = tempStore[3]

        elif (self.mode == MODE_NIETO):
            tempStore = self.estimateNIETO(Li, Mi, Lengths, MSS, 2, vp)
            Li = tempStore[0]
            Mi = tempStore[1]
            Lengths = tempStore[2]
            MSS = tempStore[3]
            vp = tempStore[4]
        else:
            print("ERROR: mode not supported. Please use {LS, LIEB, NIETO}\n")

        return Li, Lengths, Mi, MSS, vp

    def GetConsensusSet(self, vpNum, Li, Lengths, Mi, vp, E, CS_counter):

        # Compute the error of each line segment of LSS with respect to v_est
        # If it is less than the threshold, add to the CS
        for i in range (0, len(self.CS_idx)):
            self.CS_idx[i] = -1
        J = 0.
        if (self.mode == MODE_LS):
            tempStore = self.errorLS(vpNum, Li, vp, E, CS_counter)
            J = tempStore[0]
            Li = tempStore[1]
            vp = tempStore[2]
            E = tempStore[3]
            CS_counter = tempStore[4]

        elif (self.mode == MODE_NIETO):
            tempStore = self.errorNIETO(vpNum, Li, Lengths, Mi, vp, E, CS_counter)
            J = tempStore[0]
            Li = tempStore[1]
            Lengths = tempStore[2]
            Mi = tempStore[3]
            vp = tempStore[4]
            E = tempStore[5]
            CS_counter = tempStore[6]
        else:
            print ("ERROR: mode not supported, please use {LS, LIEB, NIETO}\n")

        return J, Li, Lengths, Mi, vp, E, CS_counter

    def estimateLS(self, Li, Lengths, set, set_length, vp):
        if set_length == self.minimal_sample_set_dimension:
            # Just the cross product
            # DATA IS CALIBRATED in MODE_LS

            ## This was a 3,1 array
            ls0 = np.zeros(3)
            ls1 = np.zeros(3)

            ## Check what this is about
            ls0[0] = Li[set[0], 0]
            ls0[1] = Li[set[0], 1]
            ls0[2] = Li[set[0], 2]

            ls1[0] = Li[set[1], 0]
            ls1[1] = Li[set[1], 1]
            ls1[2] = Li[set[1], 2]

            vp = np.cross(ls0, ls1)

            cv2.normalize(vp, vp)

            return Li, Lengths, set, vp

        elif (set_length < self.minimal_sample_set_dimension):
            print("Error: at least 2 line-segments are required\n")
            return

        # Extract the line segments corresponding to the indexes contained in the set
        li_set = np.zeros((3, set_length))
        Lengths_set = np.zeros((set_length, set_length))
        ##Lengths_set.setTo(0)

        # Fill line segments info
        for i in range(0,set_length):

            li_set[0, i] = Li[set[i], 0]
            li_set[1, i] = Li[set[i], 1]
            li_set[2, i] = Li[set[i], 2]

            Lengths_set[i, i] = Lengths[set[i], set[i]]

        # Least squares solution
        # Generate the matrix ATA(apartirde LSS_set = A)
        L = np.transpose(li_set)
        Tau = Lengths_set
        ATA = np.zeros((3, 3))
        ATA = np.transpose(L) * np.transpose(Tau) * Tau * L

        # Obtain eigendecomposition

        w = np.zeros(1)
        v = np.zeros(1)
        vt = np.zeros(1)
        ## check if w,v and vt needs to be initiated
        cv2.SVDecomp(ATA, w, v, vt)

        # Check eigenvecs after SVDecomp
        ## check at runtime. needs to output the number of rows
        if (len(v) < 3):
            return

        # Assign the result(the last column of v, corresponding to the eigenvector with lowest eigenvalue)
        vp[0, 0] = v[0, 2]
        vp[1, 0] = v[1, 2]
        vp[2, 0] = v[2, 2]

        cv2.normalize(vp, vp)

        return

    def estimateNIETO(self, Li, Lengths, Mi, set, set_length, vp):
        if (set_length == self.minimal_sample_set_dimension):
            # Just the cross product
            # DATA IS NOT CALIBRATED for MODE_NIETO

            ## 3x1 vectors
            ls0 = np.zeros(3)
            ls1 = np.zeros(3)

            ls0[0] = Li[set[0], 0]
            ls0[1] = Li[set[0], 1]
            ls0[2] = Li[set[0], 2]

            ls1[0] = Li[set[1], 0]
            ls1[1] = Li[set[1], 1]
            ls1[2] = Li[set[1], 2]

            vp = np.cross(ls0, ls1)

            # Calibrate( and normalize) vp
            vp = np.linalg.inv(self.K) * vp
            cv2.normalize(vp, vp)
            return Li, Lengths, Mi, set, vp

        elif (set_length < self.minimal_sample_set_dimension):
            print("Error: at least 2 line-segments are required\n")
            return Li, Lengths, Mi, set, vp

        # Extract the line segments corresponding to the indexes contained in the set
        li_set = np.zeros((3, set_length))
        Lengths_set = np.zeros((set_length, set_length))
        mi_set = np.zeros((3, set_length))
        ##Lengths_set.setTo(0)

        # Fill line segments info
        for i in range(0, set_length):
            li_set[0, i] = Li[set[i], 0]
            li_set[1, i] = Li[set[i], 1]
            li_set[2, i] = Li[set[i], 2]

            Lengths_set[i, i] = Lengths[set[i], set[i]]

            mi_set[0, i] = Mi[set[i], 0]
            mi_set[1, i] = Mi[set[i], 1]
            mi_set[2, i] = Mi[set[i], 2]

        dtheta = 0.01
        dphi = 0.01

        numTheta = int(np.pi / (2 * dtheta))
        numPhi = int(2 * np.pi / dphi)
        ##CHECK!
        debugMap = np.zeros(numTheta, numPhi)
        #debugMap.setTo(0)

        #data_struct = namedtuple("LSS", "Lengths", "midPoints", "K")

        dataTest = namedtuple("LSS", "Lengths", "midPoints", "K")
        dataTest = (li_set, Lengths_set, mi_set, self.K)
        #double * fvecTest = new double[set_length]
        #int *infoTest
        #aux = 0
        #infoTest = &aux

        fvecTest = np.zeros(set_length)
        aux = 0
        infoTest = aux

        # Image limits
        pt0 = np.zeros(3)
        pt3 = np.zeros(3)
        pt0[0] = 0
        pt0[1] = 0
        pt0[2] = 1
        pt3[0] = self.width
        pt3[1] = self.height
        pt3[2] = 1

        pt0C = np.linalg.inv(self.K) * pt0
        cv2.normalize(pt0C, pt0C)
        pt3C = np.linalg.inv(self.K) * pt3
        cv2.normalize(pt3C, pt3C)

        theta0 = np.arccos(pt0C[2, 0]) #double
        phi0 = np.arctan2(pt0C[1, 0], pt0C[0, 0])
        print("\nPt0(sph): (%.2f, %.2f)\n", theta0, phi0)

        theta3 = np.arccos(pt3C[2, 0])
        phi3 = np.arctan2(pt3C[1, 0], pt3C[0, 0])
        print("Pt3(sph): (%.2f, %.2f)\n", theta3, phi3)

        paramTest = [0, 0] #paramTest[] = {0, 0}
        maxE = 0
        minE = FLT_MAX

        for t in range(0, numTheta):
            theta = dtheta * t
            for p in range(0,numPhi):
                phi = dphi * p - np.pi
                paramTest[0] = theta
                paramTest[1] = phi

                evaluateNieto(paramTest, set_length, dataTest, fvecTest, infoTest)

                for m in range(0,set_length):
                    debugMap[t, p] += fvecTest[m]

                if (debugMap[t, p] < minE):
                    minE = debugMap[t, p]
                elif (debugMap[t, p] > maxE):
                    maxE = debugMap[t, p]
        debugMapIm = np.zeros(numTheta, numPhi)
        scale = 255 / (maxE - minE)

        cv2.convertScaleAbs(debugMap, debugMapIm, scale)

        ##delete[] fvecTest

        cv2.imshow("DebugMap", debugMapIm)
        cv2.waitKey(0)

        # Lev. - Marq.solution
        m_dat = set_length
        num_par = 2

        # The starting point is the provided vp which is  calibrated
        if (self.verbose):
            print("\nInitial Cal.VP = (%.3f,%.3f,%.3f)\n", vp[0, 0], vp[1, 0],vp[2, 0])
            vpUnc = np.zeros(3)
            vpUnc = self.K * vp
            if (vpUnc[2] != 0):
                vpUnc[0] /= vpUnc[2]
                vpUnc[1] /= vpUnc[2]
                vpUnc[2] = 1
            print("Initial VP = (%.3f,%.3f,%.3f)\n", vpUnc[0], vpUnc[1], vpUnc[2])

        # Convert to spherical coordinates to move on the sphere surface(restricted to r = 1)
        x = vp[0, 0]
        y = vp[1, 0]
        z = vp[2, 0]
        r = cv2.norm(vp)
        theta = np.arccos(z / r)
        phi = np.arctan2(y, x)

        if (self.verbose):
            print("Initial Cal.VP (Spherical) = (%.3f,%.3f,%.3f)\n", theta, phi, r)

        # double par= [double (vp[0, 0]), double(vp[1, 0]), double(vp[2, 0])]
        par = [theta, phi]

        lm_control_struct control = lm_control_double
        control.epsilon = 1e-5 # less than 1�
        if (self.verbose):
            control.printflags = 2 # monitor status(+1) and parameters(+2), (4): residues at end of fit, (8): residuals at each step
        else:
            control.printflags = 0
        lm_status_struct status
        data_struct data(li_set, Lengths_set, mi_set, self.K)

        lmmin(num_par, par, m_dat, &data, evaluateNieto, & control, & status, lm_printout_std)

        if (self.verbose):
            print("Converged Cal.VP (Spherical) = (%.3f,%.3f,%.3f)\n", par[0], par[1], r)

        # Store into vp
        # 1) From spherical to cartesian
        theta = par[0]
        phi = par[1]
        x = r * np.cos(phi) * np.sin(theta)
        y = r * np.sin(phi) * np.sin(theta)
        z = r * np.cos(theta)

        vp[0, 0] = float(x)
        vp[1, 0] = float(y)
        vp[2, 0] = float(z)

        return Li, Lengths, Mi, set, vp

    def errorLS(self, vpNum, Li, vp, E, CS_counter):

        ## need deepcopy?
        vn = vp
        vn_norm = cv2.norm(vn)

        ##3x1 array
        li = np.zeros(3)
        li_norm = 0.
        di = 0.

        J = 0.
        for i in range ( 0, len(Li)):
            li[0, 0] = Li[i, 0]
            li[1, 0] = Li[i, 1]
            li[2, 0] = Li[i, 2]

            li_norm = cv2.norm(li) # estolopodriaprecalcularantes
            di = np.dot(vn, li)
            di /= (vn_norm * li_norm)

            E[i] = di * di

            #Add to CS if error is less than expected noise
            if (E[i] <= self.T_noise_squared):
                self.CS_idx[i] = vpNum # set index to 1
                CS_counter += 1

                # Torr method
                J += E[i]
            else:
                J += self.T_noise_squared

        J /= CS_counter

        return J, Li, vp, E, CS_counter

    def errorNIETO(self, vpNum, Li, Lengths, Mi, vp, E, CS_counter):
        J = 0.
        di = 0.

        lineSegment = np.zeros(3)
        lengthLineSegment = 0
        midPoint = np.zeros(3)

        # The vp arrives here calibrated, need to uncalibrate(check it anyway)
        vn = np.zeros(3)
        vpNorm = cv2.norm(vp)
        if (np.fabs(vpNorm - 1) < 0.001):
            # Calibrated -> uncalibrate
            vn = self.K * vp
            if (vn[2] != 0):
                vn[0] /= vn[2]
                vn[1] /= vn[2]
                vn[2] = 1

        for i in range(0 ,len(Li)):
            lineSegment[0] = Li[i, 0]
            lineSegment[1] = Li[i, 1]
            lineSegment[2] = Li[i, 1]

            lengthLineSegment = Lengths[i, i]

            midPoint[0] = Mi[i, 0]
            midPoint[1] = Mi[i, 1]
            midPoint[2] = Mi[i, 2]

            tempStore = distanceNieto(vn, lineSegment, lengthLineSegment, midPoint)

            di = tempStore[0]
            vn = tempStore[1]
            lineSegment = tempStore[2]
            midPoint = tempStore[3]

            E[i] = di * di

            #Add to CS if error is less than expected noise

            if (E[i] <= self.T_noise_squared):
                self.CS_idx[i] = vpNum # set index to 1
                CS_counter += 1

                # Torr method
                J += E[i]
            else:
                J += self.T_noise_squared

            J += E[i]

        J /= CS_counter

        return J, Li, Lengths, Mi, vp, E, CS_counter

    def multipleVPEstimation(self, lineSegments,lineSegmentsClusters, numInliers, vps, numVps):

        ##deepcopy needed?
        lineSegmentsCopy = lineSegments
        number_of_inliers = 0

        for vpNum in range(0,numVps):
            lineSegments = self.fillDataContainers(lineSegments)
            numLines = len(lineSegments)

            if self.verbose:
                print("VP " + str(vpNum) + "-----\n")

            if (numLines < 3 or (numLines < self.minimal_sample_set_dimension)):
                if(self.verbose):
                    print("Not enough line segments to compute vanishing point\n")
                break

            #Vector containing indexes for current vp
            ## Not sure if this needs initiating
            ind_CS = np.zeros(1)

            N_I_best = self.minimal_sample_set_dimension
            J_best = FLT_MAX

            iter = 0
            T_iter = INT_MAX
            no_updates = 0
            max_no_updates = INT_MAX

            #Define containers of CS (Consensus set): self.CS_best to store the best one, and self.CS_idx to evaluate a new candidate

            ## from vector<int>(numLines, 0)
            self.CS_best = np.zeros(numLines)
            self.CS_idx = np.zeros(numLines)

            #Allocate Error matrix
            ## vector<float>(numLines, 0)
            E = np.zeros(numLines)

            if (self.verbose):
                if (self.mode == MODE_LS):
                    print("Method: Calibrated Least Squares\n")
                if (self.mode == MODE_NIETO):
                    print("Method: Nieto\n")

                print("Start MSAC\n")

            #RANSAC loop
            while ((iter <= self.min_iters) or ((iter<=T_iter) and (iter <= self.max_iters) and (no_updates <= max_no_updates))):
                iter += 1
                if (iter >= self.max_iters):
                    break
                #Hypothesize - -----------------------
                #Select MSS

                if len(self.Li) < len(self.MSS):
                    break

                tempStore = self.GetMinimalSampleSet(self.Li, self.Lengths, self.Mi, self.MSS, self.vpAux)

                ##
                self.Li = tempStore[0]
                self.Lengths = tempStore[1]
                self.Mi = tempStore[2]
                self.MSS = tempStore[3]
                self.vpAux = tempStore[4]
                # Test - -------------------------------
                # Find the consensus set and cost
                N_I = 0
                tempStore = self.GetConsensusSet(vpNum, self.Li, self.Lengths, self.Mi, self.vpAux, E, N_I) #the CS is indexed in CS_idx

                ### == J before
                J = float(tempStore[0])
                self.Li = tempStore[1]
                self.Lengths = tempStore[2]
                self.Mi = tempStore[3]
                self.vpAux = tempStore[4]
                E = tempStore[5]
                N_I = tempStore[6]

                if (N_I >= self.minimal_sample_set_dimension) and (J < self.J_best) or ((J == self.J_best) and (N_I > self.N_I_best)):
                    self.notify = True

                    self.J_best = J
                    self.CS_best = self.CS_idx

                    self.vp = self.vpAux #Store into vp(current best hypothesis): self.vp is therefore calibrated

                    if (N_I > self.N_I_best):
                        self.update_T_iter = True

                    self.N_I_best = N_I

                    if (self.update_T_iter): #Update number of iterations
                        q = 0.
                        if (self.minimal_sample_set_dimension > self.N_I_best):
                            # Error!
                            print ("ERROR - The number of inliers must be higher than minimal sample set")
                        if (numLines == self.N_I_best):
                            q = 1
                        else :
                            q = 1
                            for j in range(0,self.minimal_sample_set_dimension):
                                q *= (self.N_I_best - j) / (numLines - j)

                        # Estimate the number of iterations for RANSAC
                        if ((1-q) > 1e-12):
                            T_iter = np.ceil(np.log(self.epsilon) / np.log((1-q)))
                        else:
                            T_iter = 0

                else:
                    self.notify = False

                # Verbose
                if (self.verbose and self.notify):
                    aux = np.max(T_iter, self.min_iters)
                    print("Iteration = %5d/%9d. ", iter, aux)
                    print("Inliers = %6d/%6d (cost is J = %8.4f)\n", self.N_I_best, numLines, self.J_best)

                    if (self.verbose):
                        print("MSS Cal.VP = (%.3f,%.3f,%.3f)\n", self.vp[0, 0], self.vp[1, 0], self.vp[2, 0])


                # Check CS length( for the case all line segments are in the CS)
                if (self.N_I_best == numLines):
                    if (self.verbose):
                        print("All line segments are inliers. End MSAC at iteration %d.\n", iter)
                    break

            # Reestimate - -----------------------------
            if (self.verbose):
                print("Number of iterations: %d\n", iter)
                print("Final number of inliers = %d/%d\n", self.N_I_best, numLines)

            # Fill ind_CS with self.CS_best
            ##Initialise??
            lineSegmentsCurrent = np.zeros(1)
            for i in range(0, numLines):
                if (self.CS_best[i] == vpNum):
                    a = i
                    ind_CS = np.append(ind_CS, a)
                    lineSegmentsCurrent = np.append(lineSegmentsCurrent, lineSegments[i])

            if (self.J_best > 0 and ind_CS.size() > self.minimal_sample_set_dimension): # if J == 0 maybe its because all line segments are perfectly parallel and the vanishing point is at the infinity
                if (self.verbose):
                    print("Reestimating the solution... ")
                    #fflush(stdout)

                if (self.mode == MODE_LS):
                    tempStore = self.estimateLS(self.Li, self.Lengths, ind_CS, self.N_I_best, self.vp)
                    self.Li = tempStore[0]
                    self.Lengths = tempStore[1]
                    ind_CS = tempStore[2]
                    self.vp = tempStore[3]
                elif (self.mode == MODE_NIETO):
                    tempStore = self.estimateNIETO(self.Li, self.Lengths, self.Mi, ind_CS, self.N_I_best, self.vp) # Output self.vp is calibrated
                    self.Li = tempStore[0]
                    self.Lengths = tempStore[1]
                    self.Mi = tempStore[2]
                    ind_CS = tempStore[3]
                    self.vp = tempStore[4]
                else:
                    print ("ERROR: mode not supported, please use LS, LIEB, NIETO\n")

                if (self.verbose):
                    print("done!\n")

                # Uncalibrate
                if (self.verbose):
                    print("Cal.VP = (%.3f,%.3f,%.3f)\n", self.vp[0, 0], self.vp[1, 0], self.vp[2, 0])
                self.vp = self.K * self.vp
                if (self.vp[2, 0] != 0):
                    self.vp[0, 0] /= self.vp[2, 0]
                    self.vp[1, 0] /= self.vp[2, 0]
                    self.vp[2, 0] = 1

                else:
                    # Since this is infinite, it is better to leave it calibrated
                    self.vp = np.linalg.inv(self.K) * self.vp

                if (self.verbose):
                    print("VP = (%.3f,%.3f,%.3f)\n", self.vp[0, 0], self.vp[1, 0], self.vp[2, 0])

                # Copy to output vector
                vps = np.append(vps, self.vp)

            elif (np.fabs(self.J_best - 1) < 0.000001):
                if (self.verbose):
                    print("The cost of the best MSS is 0! No need to reestimate\n")
                    print("Cal. VP = (%.3f,%.3f,%.3f)\n", self.vp[0, 0], self.vp[1, 0],
                           self.vp[2, 0])

                # Uncalibrate
                self.vp = self.K * self.vp
                if (self.vp[2, 0] != 0):

                    self.vp[0, 0] /= self.vp[2, 0]
                    self.vp[1, 0] /= self.vp[2, 0]
                    self.vp[2, 0] = 1

                    if (self.verbose):
                        print("VP = (%.3f,%.3f,%.3f)\n", self.vp[0, 0], self.vp[1, 0], self.vp[2, 0])

                else:
                    # Calibrate
                    self.vp = np.linalg.inv(self.K) * self.vp

                # Copy to output vector
                vps = np.append(vps, self.vp)

            # Fill lineSegmentsClusters containing the indexes of inliers for current vps
            if (self.N_I_best > 2):
                while (len(ind_CS)):

                    ##NEED TO SORT THIS OUT
                    lineSegments = np.delete(lineSegments, ind_CS[ind_CS.size() - 1], 0)
                    ind_CS = np.delete(ind_CS, len(ind_CS), 0)

            # Fill current CS
            lineSegmentsClusters = np.append(lineSegmentsClusters, lineSegmentsCurrent)

            # Fill numInliers
            numInliers = np.append(numInliers, self.N_I_best)

        # Restore lineSegments
        lineSegments = lineSegmentsCopy
        return vps #, numInliers, lineSegmentsClusters



def distanceNieto(vanishingPoint, lineSegment, lengthLineSegment, midPoint):

    # IMPORTANT: The vanishing point must arrive here uncalibrated and in Cartesian coordinates
    # Line segment normal(2 D)2
    n0 = -lineSegment[1]
    n1 = lineSegment[0]
    nNorm = np.sqrt(n0 * n0 + n1 * n1)

    # Midpoint
    c0 = midPoint[0]
    c1 = midPoint[1]
    c2 = midPoint[2]

    # Vanishingpoint(uncalibrated)

    v0 = vanishingPoint[0]
    v1 = vanishingPoint[1]
    v2 = vanishingPoint[2]

    r0 = v1 * c2 - v2 * c1 # floats
    r1 = v2 * c0 - v0 * c2 # floats
    rNorm = np.sqrt(r0 * r0 + r1 * r1) #float

    num = (r0 * n0 + r1 * n1) #float
    if (num < 0):
        num = -num

    d = 0
    if (nNorm != 0 and rNorm != 0):
        d = num / (nNorm * rNorm)

    # d *= lengthLineSegment

    return d, vanishingPoint, lineSegment, midPoint


def evaluateNieto(param, m_dat, data, fvec, info):

    ##paramTest, set_length, (const void * ) & dataTest, fvecTest, infoTest

    # Cast to correct types
    #     data_struct * mydata
    #     mydata = (data_struct *) data

    ndat = 0

    # IMPORTANT!!: the vanishing point has arrived here calibrated AND in spherical coordinates!
    # 1) Get Cartesian coordaintes
    theta = param[0] #double
    phi = param[1] #double
    x = np.cos(phi) * np.sin(theta) #double
    y = np.sin(phi) * np.sin(theta) #double
    z = np.cos(theta) #double

    # 2) Uncalibrate it using the K matrix in the data

    vn = np.array(3)
    vn[0] = x
    vn[1] = y
    vn[2] = z

    vanishingPoint = mydata->K * vn #matrix
    if (vanishingPoint[2] != 0):
        vanishingPoint[0] /= vanishingPoint[2]
        vanishingPoint[1] /= vanishingPoint[2]
        vanishingPoint[2] = 1


    lineSegment = np.zeros(3)
    lengthLineSegment = 0
    midPoint = np.zeros(3)

    # Fill fvec
    for p in range(0,mydata->LSS.cols):

        # Compute error for this vanishing point (contained in param), and this line segment (LSS.at < double > (0, p)LSS.at < double > (1, p)LSS.at < double > (2, p)) and midPoint
        lineSegment[0] = mydata->LSS[0, p]
        lineSegment[1] = mydata->LSS[1, p]
        lineSegment[2] = mydata->LSS[2, p]

        lengthLineSegment = mydata->Lengths[p, p]

        midPoint[0] = mydata->midPoints[0, p]
        midPoint[1] = mydata->midPoints[1, p]
        midPoint[2] = mydata->midPoints[2, p]

        tempStore = distanceNieto(vanishingPoint, lineSegment, lengthLineSegment, midPoint)
        fvec[p] = tempStore[0]
        vn = tempStore[1]
        lineSegment = tempStore[2]
        midPoint = tempStore[3]

    #to prevent a 'unused variable' warning
    info = info

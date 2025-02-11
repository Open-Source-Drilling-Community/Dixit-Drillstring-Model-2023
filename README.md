# EM-Coupled-Axial-Torsional-Model
ExxonMobil Coupled Axial Torsional Model 2023-06

 
Preface

Rotary drilling systems exhibits self-excited coupled axial and torsional vibrations in the drill-string under various operating conditions of drilling. The severity of these dysfunctions leads to the onset of Stick-slip vibration with PDC bits. These vibrations are governed by the torsional stick and slip phases of the drill-string. Periodic stick-slip phases results in zero bit angular velocity, where string stores the reactive torque momentarily and releases the trapped torque in slip phase, where the angular velocity peaks to twice or thrice the average value.

Stick-slip vibrations have negative effects on the drilling performance as it reduces the efficiency and even leads to bit and BHA wear in severe conditions. It is widely accepted in the drilling industry that the root cause of stick-slip oscillations is primarily due to bit-rock interaction. In order to mitigate these vibrations, the understanding of bit-rock interaction and drill-string dynamics in the presence of friction is of interest.

To date, ExxonMobil has existing In house Vybs capability to tackle lateral vibrations and for governing axial-torsional vibrations, TORAX has been widely used. Although, the requirement of a transient capability tool to govern the drill-string dynamics behavior has always been felt over the course of time so as to assist various Wells teams in planning their future wells.

Current drilling simulator is the first of its kind that assists in modeling transient drill-string behavior under different tripping and drilling operations. The tool helps to understand the severity of vibrational dysfunctions on the drilling performance and assist in providing recommendations to various BUs including Qatar, Permian and Guyana.

This document provides detailed review of technical information gathered on Drill-string dynamics models and captures the current drilling simulator workflow developed to model multiple drilling scenarios and tackle vibrational dysfunctions. I hope you find it informative and useful.  If you require assistance to model your drill-string to understand the vibrational severity while drilling based on offset wells performance, feel free to reach out to us.


Have a nice day,

Rajat Dixit


 



 



Contents

Preface	1
Part-I: Theory and Background	3
Introduction	4
Bit-rock Interaction	5
Part-II: Drilling Simulator Methodology	6
Drilling Simulator: Methodology	7
Friction Model	15
Special Cases: Drilling Simulator in Absence of Mud-motor	19
Mud-motor Integrated Drilling Simulator: Methodology	21
Special Cases: Mud-motor Integrated Drilling Simulator	26
Part-III: Drilling Simulator Features	39
Drilling Simulator: Developed Features	40
Drilling Simulator: Under Development Features	42
Appendix	43
Drilling Simulator: Code Functionality	44



 








Part-I: Theory and Background 
Introduction

Drilling dynamics describes the behavior of the drilling system while creating the borehole. The drilling system itself contains sub-systems and components, such as the drill bit, rotary steerable, motor, reamer, Drill-string, surface and other equipment, as well as the hole geometry and formation properties. Dynamics modeling focuses on discrete components in this system and coupling of these discrete models. As illustrated by Rudat and Dashevskiy, 2011, a simple one-degree-of-freedom rotational lumped-parameter model of the drill-string, when coupled with a simple falling-friction model of the drill-bit (more on this in the bit-rock interaction section), yields a powerful drilling dynamics model of stick-slip for use in control system development.

Majority of existing technical papers published on drilling dynamics provide mathematical details of models with implemented assumptions and solution methodology. Most of the work being shared does not include codes for extensive reviews. A brief summary of various model types and solution methodology has been segregated in Figure 1 comprising of multiple levels of complexity that have been used industry wide.

 
Figure 1: Different Model Types and Range of Complexity (Ref: SPE/IADC-194082-MS)

Degrees of Freedom (DOF) are the number of variables that determine the state of the physical system (Figure 3). DOF can vary from a lumped-parameter model with one DOF (a point mass on a line, e.g. Rudat and Dashevskiy, 2011), to multi-body models which contain several DOF (Abbassian and Dunayevkiy, 1998 extended a lumped parameter model to include lateral motion). Lumped parameter models consist of rigid masses separated by kinematic pairs of dashpots, and are a function of time only. Bailey et al, 2008 describe a 2D model made of one-foot bodies (so many DOFs) connected by "massless beam spring elements" which can describe lateral displacement and bending moments. The other end of the spectrum is distributed-parameter models (e.g. Dykstra et al 2001), in which the parameters are distributed in both time and length along the drill-string and wave propagation is taken into account. Distributed parameter models can have many DOF's and can be very complex.

A classification of drilling dynamics models by use is difficult, but current areas of use include day-to-day engineering, design of bottom-hole-assemblies and components, drilling dynamics research and analysis, and automation control. This classification depends partly on complexity and partly on function. Efficient engineering models are those which are sufficiently fast, or which have specific output used in day-today operations and design work" (Dykstra et al, 2001). For example, a finite element code in the frequency domain is for engineering work, while research will use the slower time domain code, consisting of coupled nonlinear equations of motion. The Dykstra et al, 2001 paper is another good example of coupling a kinematic bit model with a complex model of the drill-string. An emerging use for drilling dynamics models is to couple these with control systems to mitigate dysfunctions and extend drilling capabilities.

Empirical rules and driller's experience are often used to dictate the choice of the operating parameters. It is now recognized that optimum drilling performance can best be achieved by combining real-time measurements of drilling parameters with an interpretation scheme based on the interface laws that describe the interaction between the bit and the rock.

Note: A detailed literature review of various existing models in the field of drilling dynamics has been carried out to lay the foundation of current drilling simulator. (Link: Drilling Dynamics Literature Review)

Bit-rock Interaction

A key factor in common with the above referenced literature is that the bit-rock interaction is used to explain the cause of stick slip. The bit-rock interface laws for drilling primarily depends on the relationships of Weight on bit, torque on bit, rate of penetration and angular velocity. The coupled effects of these variables governs the efficient drilling and also set the insurgence of drilling dysfunctions into the system. The force and torque on bit will have dependency on the instantaneous change in axial and angular velocities. The PDC bit-rock interaction is a combination of cutting and frictional contact (Detournay and Defourny 1992). The WOB and Torque on bit are further decomposed into cutting and frictional components depending on the rock strength of formation to be drilled. 

The cutting components will have direct dependency on the depth of cut whereas the frictional components will be accounted to keep track of the dissipated lost energy in model while drilling the formation. To better assess the frictional forces at bit and along the length of drill-string in a deviated wellbore, a consolidated friction model (IADC/SPE-199678) is implemented in the drilling simulator. 











Part-II: Drilling Simulator Methodology  
Drilling Simulator: Methodology

Current Drilling simulator is a coupled 2-DOF Axial-torsional system to model the transient dynamics of drill-string under different operating conditions of drilling and tripping. Modeling is carried out as a Lumped-parameter study. Figure 2 and 3 highlights the Model Schematics along the axial and rotary directions. The drill-string is segregated into multiple elements comprising of drill-pipes, HWDP and collar elements. 
The bit rock interaction is governed by a depth of cut based model. Stribeck friction is utilized to account for the Coulomb friction, whereas, borehole friction is accounted using Viscous frictional force. The net frictional force have dependency on the gravity force, drilling mud and well inclination. Reaction forces due to bit-rock interaction will be present for the duration bit is on bottom. Following geometrical based parameters of drill-string elements are required as model input for both single and multi-elements drill-string system:
	Element mass 
	Mass Moment of Inertia
	Area of Cross-section
	Axial Stiffness
	Torsional Stiffness
Model Axial and Rotary Schematics:
 
Figure 2: Drilling Simulator Axial Motion Schematic
 
Figure 3: Drilling Simulator Rotary Motion Schematic

	Drill-String Model Parameters:

	Effective Mass Moment of Inertia (J):  Mass moment of inertia is being calculated about the Z axis as,
J=1/4×m×(D_o^2+D_i^2 )
Where, m is the lumped mass of the drill-string element under consideration. 
	Area of Cross-section (A):  Area of Cross-section of Drill-String elements is being calculated as,
A=π/4×(D_o^2-D_i^2 )
	Axial Stiffness(k_a):  Axial stiffness is being calculated using Area of cross-section of pipes (A), Young’s Modulus of Elasticity (E) and length of Drill-string (L), respectively as,

k_a=((A×E))/L

	Torsional Stiffness(k_t):  Torsional stiffness is being calculated using Polar Moment of Inertia (A), Shear Modulus (G) and length of Drill-string (L), respectively as,

k_t=((G×J))/L
Where,
J=  π/32×(D_o^4-D_i^4 )

Note: For Steel, E = 210 GPa and G = 79.3 GPa 

	Calculations of Model Input parameters:

Parameters	Well 1	
Well 2
Section TD	5864 m	
2762 m
Drill-pipe OD	6.625’’	
5.875’’
Tool-joint Size	8.5’’	
7’’
Hole Section (Diameter)	12.25’’	
12.25’’
CCS formation	2000-60000 Psi	
2000-60000 Psi
Table 1: Model Input Parameters

	Steady-State Parameters for 2000 Psi CCS Value

	〖WOB〗_SS = 1500 lbs
	〖ROP〗_SS = 27 m/hr.
	〖RPM〗_SS = 120 rpm
	〖DOC〗_SS =  〖ROP〗_ss/〖RPM〗_ss  = 0.147 inch/rev
Note: Values of 〖RPM〗_SS, 〖ROP〗_SS and 〖DOC〗_SS are held the same for all simulation runs to have uniformity and robustness in model for different sensitivity cases. Initial model tuning parameters is taken to be conservative to setup the problem in simulator.
	Calculation of Other Model Inputs 

	Top-drive Axial Velocity of Draw-works (V_o)

	Top-drive Angular Velocity of Drum (ω_o)

Coefficient of Friction of Rock (μ_Rock):

	Linear empirical relation between rock cutting parameter μ_Rock and confined compressive strength of rock CCS is as follows:

μ_Rock=(-0.0201×CCS)+1.5333 

 

Figure 4: Relationship between μ_Rock and Rock strength (Ref: Dave Schnell, BHGE)

Note: CCS of formation limits to 60 ksi to give stable robust solutions, as beyond CCS = 76 ksi, value μ_Rock goes negative.
	An alternate empirical relation on logarithmic scale is created to avoid limitation on rocks having higher confined compressive strength.
 

	Coefficient of WOB (k_WOB): 

k_WOB=〖WOB〗_(Cutting Component)/〖DOC〗_bit 

k_WOB=(0.8 )×(0.5×CCS)×(1500*4.45)×(1/〖DOC〗_ss )×(〖Dia〗_Hole/(12.25))
where, 0.5  is model constant for Rock Strength with units in (1/ksi) 
Note: k_WOB is calculated assuming a linear model. Value of k_WOB is also a function of CCS of formation. 

	Coefficient of Torque (k_TQ):
k_TQ=〖TQ〗_bit/〖DOC〗_bit 
k_TQ=(k_WOB )×(〖Dia〗_Hole )×(μ_Rock/3)

Rock Strength (Psi)	μ_Rock
2000	1.4931

6000	1.4127

20000	1.1313

40000	0.7293

60000	0.3273

Table 2: Coefficient of Friction of Rock Vs Rock Strength

〖ROP〗_bit	〖RPM〗_bit	〖Reaction Torque〗_bit	〖Reaction Force〗_bit
+	0		k_WOB×〖DOC〗_(current iteration )
-	0		k_WOB×〖DOC〗_(current iteration )
0	+	k_TQ×〖DOC〗_(current iteration )	
0	-	〖-k〗_TQ×〖DOC〗_(current iteration )	
Table 3: Bit Reaction Force and Torque components variation with sliding velocities
Axial and Rotary Borehole Viscous Damper: The Viscous damping effects due to mud flow in the borehole is also accounted in the simulator,
Under Steady-State conditions, we get,

	c_(t-borehole)=7500-20000 

	c_(a-borehole)=c_(t-borehole)×(〖ROP〗_ss/((〖RPM〗_SS )×(π×D_(Tool-joint) ) ))


Note:  
	Units of c_(t-borehole), c_(a-borehole) and c_(borehole-resultant) in  ‘N-sec/m’

	Drilling simulator accounts for the well path trajectory and variation of TQ along the string length to include the distribution of dynamics along the string.
	Values of Viscous dampers are held constants during simulation but the Viscous frictional force will vary depending on the axial and tangential sliding velocities
GOVERNING EQUATIONS:
mZ ̈+k_a (Z-Z_o )+F_(Friction (Coulomb+Viscous))+F_Formation Reaction=0
Jθ ̈+k_t (θ-θ_o )+〖TQ〗_(Friction (Coulomb+Viscous))+〖TQ〗_Formation Reaction=0   

F_Formation Reaction=〖WOB〗_downhole
〖TQ〗_Formation Reaction=〖TQ〗_downhole
〖WOB〗_downhole= (F_Formation Reaction Cutting Component ) + (F_Formation Reaction Frictional Component )

In Vector form above equations takes the form:


























Where,

	Z_o= ∫▒〖V_0 dt〗
	θ_o= ∫▒〖ω_0 dt〗
	m_n = drill string element mass
	Z = bit position downward direction considered as positive
	k_a = axial stiffness of drill-string element
	V_o = Top-drive (draw-works) axial velocity as input
	Z_(top_drive)= Top-drive (draw-works) axial displacement as input
	Z ̇ = bit axial velocity downward direction considered as positive
	〖WOB〗_Surface = Surface/Top-drive WOB (In ROP Control Mode)
	J_n = mass moment of inertia of drill string element
	θ = rotation angle, Clockwise direction is considered as positive
	ω_o = Topdrive angular velocity as input 
	θ_(top_drive)= Topdrive rotation as input 
	k_t = torsional stiffness of drill-string
	θ ̇ = bit rotary velocity Clockwise direction considered as positive
	〖TQ〗_Surface = Surface/Top-drive Torque 
 
Figure 5: Topdrive Axial and Rotary velocities Vs Time

Modeling Algorithm and Initial Conditions:
Coupled Axial-torsional second order differential equation sets, are solved using ODE45 solver. ODE solver is a versatile solver that implements a Runge-Kutta method with a variable time step for efficient computation. The current solver provide robust solution for an Initial valued problem.
	Z ̇=V 
	Z ̈=V ̇ = a 
	θ ̇=ω 
	θ ̈=(ω" " ) ̇= α

Equations to be used in Solver:
dZ/dt=V
dV/dt=-(1/m)×(Net External Force)
dθ/dt= ω

dω/dt=-(1/J)×(Net External Torque)
Initial Conditions for first iteration: 
Four outputs that we are getting from the equations are displacement (Z), axial velocity (Z ̇), rotation angle (Θ) and angular velocity (ω) for each elements. Following initial conditions are implemented for solving the above set of equations for first iteration.
	t  = 0
	Z  = 0
	Z ̇ = 0 
	Θ = 0
	ω = 0
	〖DOC〗_initial = 0

Downhole bit parameters
                        
	Downhole Bit WOB: 

WOB_bit= (k_WOB×〖DOC〗_(iteration ) )+(c_(bit-axial)×Z ̇_(bit-iteration) )

	Downhole Bit Torque: 

〖TQ〗_bit= (k_TQ×〖DOC〗_(iteration array) )×(±1)
 
Figure 6: Rock surface tracking feature while drilling in current drilling simulator (Bit displacement along downward direction)
Friction Model

Modeling friction forces along the length of drill-string is challenging due to the non-linear behavior of system with ever changing velocity direction. Net frictional force is a strong function of the sliding velocity. The axial component of the frictional force contributes to the drag force whereas, the tangential component of the frictional force generates a frictional torque in the system.
As the force of friction increases, the friction force introduces a negative damping effect when the sliding velocity gets close to zero. This negative damping generates stick-slip dysfunction even in the case when bit is off-bottom. In deviated wells, the topdrive axial velocity of draw-works and rpm of drum does not get transferred to the bit instantly, the elements of the drill-string have to overcome the static limits of friction to transfer the energy to the next element. This lag of energy transfer is generated effectively by the Stribeck curve, where net frictional torque decays with increase in sliding velocity.
To better understand the dynamics of system with the introduction of the friction model, model input parameters are calculated below to provide overview of the various forces and torque components acting on different drill-string elements in the axial and tangential directions.

Buoyant Factor (B_f):  
B_f=(ρ_s-ρ_m)/ρ_s 
Where,
	ρ_s is density of drill-pipe/BHA material (Steel: 7840  kg/m^3 ) 
	ρ_m is density of drilling mud (NAF based: 1350 kg/m^3 )
	From here, B_f = 0.83 (B_f is a function of the density of pipe and mud-weight)
Normal Force (F_(n,iteration)): 
F_Normal=(B_f×M_(BHA-effective)×g×sin⁡θ )
	θ= inclination angle of wellbore (in °), including well trajectory for general case (discrete distribution of drill-string elements length Vs Inclination Angle)
	g = acceleration due to gravity (9.81m/〖sec〗^2 )
	B_f= Buoyant factor accounting the density effect of Mud and BHA
Note: String Capstan effects (due to pipe stretch) are not accounted in the model yet. This feature is already present in Torque and Drag module of EMWells ToolPro.
Dynamic Frictional Force (F_(n,k)): 
F_(μ,k)=μ_k×F_n×(Sign(V_Sliding ))
The dynamic friction force will act in direction opposite to the axial velocity of motion. If, static friction applies, i.e., V_Sliding=0, then static friction force F_(μ,s) acts in the direction opposite to the sum of all tangential forces with a magnitude that equals the sum of all tangential forces. Condition below is valid for the case when V_Sliding=0
F_(μ,s)+∑▒F_Ext =0
If, static friction calculated above is less than (μ_s×F_n), where μ_s>μ_k then the surface do not slide.
Also, to avoid discontinuity at V_iteration tends to 0, we use Stribeck friction model (1902). To account this, Tustin (1947) proposes the following formulation:
μ_effective=μ_(mud-dynamic)+((μ_(mud-static)-μ_(mud-dynamic) )×e^(-(|〖Sliding Velocity〗_Resultant |/V_CS ) ) )

Note: μ_effective and  V_CS (Stribeck critical velocity) accounts for velocity weakening effect into model dynamics


  
Figure 7: Friction Model (a) SPE-199678-MS (b) Friction Model of current drilling simulator 
Mud Type	Dynamic Friction Coefficient(μ_k)	Static Friction Coefficient (μ_s)
Typical Oil based	0.2	0.25
Micronized Oil-based	0.15	0.2
KCI Polymer Water-based	0.3	0.4
Table 4: Typical values for Static and Dynamic Friction Coefficients
Friction Model Input Parameters

	〖Dia〗_(Pipe-equivalent) is used in the model to compensate for the size difference between drill-pipe and Tool-joint:
〖Dia〗_(Pipe-equivalent)=((27×〖Dia〗_pipe )+(3×〖Dia〗_(Tool-joint) ))/30
	Coulomb Friction Force:

Coulomb Friction Force=μ_effective×F_Normal
	Coulomb Friction Force Axial Component:

〖Coulomb Friction Force〗_Axial=(〖Sliding Velocity〗_(Axial Component)/|〖Sliding Velocity〗_Resultant | )× Coulomb Friction Force

	Coulomb Friction Force Tangential Component:

〖Coulomb Friction Force〗_Tangential=(〖Sliding Velocity〗_(Tangential Component)/|〖Sliding Velocity〗_Resultant | )× Coulomb Friction Force
	Coulomb Friction Torque Tangential Component:
〖Coulomb Friction Torque〗_Tangential=〖Coulomb Friction Force〗_Tangential× 0.5×〖Dia〗_(Pipe-equivalent)

	Viscous Friction Force Axial Component:

If pumps are on, then there will be an axial drag accounting for the sliding relative velocity between the pipe velocity and fluid velocity. Currently, V_Mud=0.

If pumps are on,

〖Viscous Force〗_Axial=f(V_Axial,V_Mud)
i.e.
〖Viscous Force〗_(Axial-pipe)=〖ca〗_(borehole-viscous)×(V_Axial )×(((D_pipe^2 ))/((D_hole^2 )-(D_pipe^2 ) ))


	Viscous Friction Force Tangential Direction Component:

〖Viscous Force〗_Tangential=〖ct〗_(borehole-viscous)×(ω×0.5×〖Dia〗_(Pipe-equivalent) )

〖Viscous Torque〗_Tangential=〖Viscous Force〗_Tangential×(0.5×〖Dia〗_(Pipe-equivalent) )

	Axial and Tangential Components of Net Frictional Force and Torque: 

〖Net Frictional Force〗_Axial=(〖Coulomb Friction Force〗_Axial )+(〖Viscous Friction Force〗_Axial )


〖Net Frictional Force〗_Tangential=(〖Coulomb Friction Torque〗_Tangential )+(〖Viscous Friction Torque〗_Tangential )


Note: 

	Units of c_(t-borehole) and c_(a-borehole) in ‘N-sec/m’

	The order of viscous damping constants (c_(t-borehole) and c_(a-borehole)) which acts as a model-tuning parameters, is kept consistent to keep the system in ideal damp state depending on the field data.




	To account for the borehole dampers distribution in case of multi-element string, we consider to take the damper values for elements in fraction of their mass contributing, i.e., 50% for Collars, 30% for HWDP and 20% for the drill-pipes. This can be further optimized to be more realistic in alignment with field data.

	Numerical Data presented in the document has been taken from Eric Cayeux’s work: SPE- 199678-MS, “Analysis of Torsional Stick-slip Situations Observed with Downhole High Frequency Magnetometer Data”





























Special Cases: Drilling Simulator in the Absence of Mud-motor

Well: Hebron P1D-OP17 12.25” hole-section
Max Well Inclination: 60^0
Drill-string Length: ~2600 m.
Formation CCS: 20 ksi 
Drill-string Elements: 5 (3 DP + 1 HWDP + 1 Collar)
Case 1: Drill-string dynamics under different Drilling and Tripping operations phases
 
Figure 8: Surface and downhole parameters variation during various phases of drilling operations
 
Figure 9: Variation of Axial velocity of different elements during various phases of drilling operations
Figure 8 and 9 shows the variation of surface and downhole parameters of drill-string under different phases of drilling operations,
	Phase 1 Drilling – In Phase 1, topdrive axial velocity and RPM are ramped to 27 m/hr. and 120 respectively, and allows to achieve steady-state conditions. Drilling continued for 120 seconds when all elements demonstrate steady-state behavior.
	Phase 2 Topdrive Stationary Axially – Topdrive is bring to rest and not allowed further axial motion, RPM from drum continued to be provided to the drill-string and allowed system to achieve steady-state for next 30 Seconds.
	Phase 3 POOH with Rotation – Topdrive is picked up with 27 m/hr. axial velocity and string is POOH with rotation and allowed to achieve steady-state for remaining session of the run.
It is evident from Figure 9, that it takes approx. 110-120 Seconds for the topdrive energy to get transferred to bit, as the friction forces at different elements of the drill-string have to break over the Static friction before they initiate movement and transfer energy to the element below. Once, in Phase 3, no reaction forces will act on bit that were pre-dominant during the Phase 1, and this reduces the lag of energy transfer between the elements of drill-string.
Case 2: Axial Stiction along the length of drill-string in the absence of Topdrive drum RPM during Tripping in hole
 
Figure 10: Variation of Axial velocity of different elements while tripping in hole in the absence of drum RPM
Figure 10, shows the variation of axial velocity at different drill-string elements including drill-bit while tripping in hole, in the absence of drum RPM. The axial stick and slip phases confirms that the axial drag will be pre-dominant in the absence of rotary speed. String elements have to undergo higher loads to break the static friction and transfer energy to the element below. This phenomenon was not present in previous case, as rotary speed helps to counter the axial drag during tripping and drilling phases and provide ease for element motion along the well-path. This stick effect will be severe in deviated wells in comparison to vertical wells.
Mud Motor Integrated Drilling Simulator: Methodology

Formulation of coupled Axial-torsional drilling simulator lumped parameter model has been documented in earlier section.  This section accounts for the integration of downhole motor into existing drilling simulator model.
Integrated Downhole Motor (PDM):
PDM has been extensively used in the drilling operation. The performance of the PDM can be described by pressure drop versus torque and RPM. Depending on the type of motor, the performance curves for RPM and flow rate will be different. PDM is a hydraulically driven equipment, the performance of which depends on the cross-sectional flow area between rotor and stator, rotor-stator lobe ratio, stage length, number of stages and flow rate.
The performance parameters of PDM includes output Torque, RPM, pressure drop and mechanical power. For a given type of PDM, the RPM is proportional to the flow rate pumped through the PDM whereas, the pressure drop across the motor is proportional to the Torque output.
〖Mechanical Power〗_Motor=Torque×RPM
〖Hydraulic Power〗_Motor=∆P×FlowRate
An advantage of PDMs is that output performance (T, N) can be monitored from the rig floor by tracking standpipe pressure and flow rate. The difference between off-bottom and on-bottom standpipe pressures represents pressure drop across the motor during drilling for a given weight on bit (WOB). Flow rate through the drilling system can be translated into output rotary speed. Most PDMs feature a bend sub that serves the purpose of directional drilling. With a bend hole (usually less than 4^0), a PDM is capable of drilling to the direction that the bit is pointed to.
Driven by mud flow, the power section at the top of the PDM can generate Torque and RPM on the lower end and drive the drill bit. In motor operation, there are two operation modes: Rotary and Sliding.
 
Figure 11: Mud Motor Steerable Assembly (Ref: Zoucheng Dongyuang Petroleum Co. Ltd.)
Rotary Mode: In rotary mode, the drill-string is rotated by the Topdrive, and the bend sub rotates with the upper part of the drill string. Under rotary mode, the motor is not able to steer as the bend angle is constantly changing. The bit rpm is equal to the surface rpm plus the motor rpm. 
 
Figure 12: Slide Vs Rotate (Ref: Warren, 2019. Technology gains momentum: Rotary-steerable technology Part 1.)
Sliding Mode: In Sliding mode, there is no surface rpm, and the bend sub tool-face is set up pointing towards the desired direction. The rotary speed provided by the motor power section drives the bit to drill towards the direction determined by the bend. (Ref: IADC/SPE-180591-MS)
The bit sits on a bent housing, and therefore does not point straight ahead. This causes a side force, which allows the bit to drill a curved hole. The near bit stabilizer is smaller than the bit itself, this stabilizer forms a fulcrum, within which the motor acts as a lever, so that side force may be generated at the site of the bit.
Dump or Bypass Valve: The by-pass valve, or dump valve, is used to allow drilling fluid to fill the drill-string from the annulus when it is tripping into the wellbore, or to drain it when it is tripping out. Thanks to this valve, the bottom of the wellbore maintains a constant pressure, which helps to prevent control problems during trips.
In the power section of PDM, the spiral shaped rotor produces rotation when the drilling fluid force acts upon it. Should the bit/formation resistance to rotation (known as the drilling torque requirement) be too great, then the drilling fluid can potentially cause the elastomeric material of the stator to become deformed temporarily. The division or seal between high and low pressure is then lost, which causes the motor to stall. 
As the pressure inside each cavity decreases from leakage of fluid volume past the lost seal, there will be a significant pressure increase at the surface. This means that the motor needs to be lifted off the bottom, and then restarted. Should the stall not be properly corrected, the stator will be permanently damaged, and the life of the overall motor reduced. This is especially important when working with higher flow rates or high differential pressures. Less applied differential pressure will mean fewer stalls.

Note: One stage is defined as one complete helical rotation, or pitch of the stator/housing. As the number of stages increases, the differential pressure and the torque also increases. Additionally, as the lobe count increases, the final drive speed decreases accordingly.
We are not accounting for the Surface SPP variation in the model for now. May be at a later stage. In the model, differential pressure across motor will be accounted as the SPP at motor. 


Note: To account for the total SPP at surface, following two conditions needs to be added to model as per requirement at a later stage:
	Fluid Compressibility
	Total Surface SPP = 〖SPP〗_(Off-bottom)+ 〖SPP〗_Motor
Where, 〖SPP〗_Motor is the differential pressure across motor that will be dependent on the formation reaction torque and WOB, DOC, ROP, formation CCS among few parameters.
〖SPP〗_(Off-bottom) is the differential pressure calculation during tripping mode and can be related by squared low of flow across pipe in Laminar flow,
SPP∝(〖Flow Rate〗^2 )
Also, Bit RPM will be an output based on the TQ and RPM of the motor + stator (housing)
 Equations Governing Motor Dynamics: Following parameters are to be utilized while integrating PDM (Mud Motor) functionality in the existing model.
	Bit RPM: Bit RPM comprises of the Rotor(Mud Motor) RPM and Stator (Housing) RPM
〖RPM〗_Bit=〖RPM〗_Rotor+〖RPM〗_Stator
〖RPM〗_Rotor=C_1×FlowRate×[1+(C_2×(〖TQ〗_Bit+〖TQ〗_(Lower BHA) )  )+(C_3×〖(〖TQ〗_Bit+〖TQ〗_(Lower BHA))〗^2  )  ]
〖∆P〗_Output=C_4×(〖TQ〗_Bit+〖TQ〗_(Lower BHA) )
Note: C_2 and C_3 values are negative to account for the RPM decay as reaction torque increases. In case where the PDM position is not at the bit but somewhere in the middle,  〖TQ〗_(Lower BHA) comprises of all the elements below the PDM else where the PDM position is right above the bit then 〖TQ〗_(Lower BHA) value will be zero. Currently, all model validation is performed for the cases where PDM is right above bit.

Model Parameters Calculations: 
	C_1=Rev/gallon (RPG)
	C_4=(Max.Operating ∆P(Stall Pressure))/(TQ at Max.Operating ∆P (Stall TQ))  

Note: Values of constants C_2 and C_3 accounts for the motor by-pass valve effect.
 

Figure 13: Mud motor Integrated Drilling simulator Schematic

 
Figure 14: Mud motor Dynamics

Constant Values for Current PDM Case: 6.75” Scout 7/8 5.0 
	C_1=0.28 rev/gallon
	C_2=0   (From Performance Chart C_2=-1.65×10^(-3)   RPM/(N-m) )
	C_3=0   (From Performance Chart C_3=-2.7×10^(-6)   RPM/(N-m)^2  )  
	C_4=(1130 (Psi))/(10460 (lbs-ft)) or (1130×6894.76 (Pa))/(10460×4.45×12×0.0254 (N-m))=549.150(1/m^3 )
	No. of Lobes:
	Rotor: 7,	Stator: 8
	Flow Range: 300-600 (Gallon/min), 		
	Max. Operating Diff. Pressure: 1130 Psi
	Torque at Max Operating Diff. Pressure: 10460 (lbs-ft.) 
	Bit to Bend: 4.05 ft.
	RPG: 0.28 (Revs/Gallon)
	Mud flow rate does not account for the pressure pulse in the system, the energy transfer delay from mud-pumps to mud-motor downhole is accounted by various ramp function inputs, i.e.,
Q_(Mud-pumps)=f(time)


















Special Cases: Mud-motor Integrated Drilling Simulator

Well: Precision 580 Corral Canyon 9-4 FED 122H 7.875” hole-section
Max Well Inclination: 〖90.3〗^0
Drill-string Length: 16600 ft.
Drill-bit Blades: 3 (All symmetrically placed with 120 deg. apart)
Formation CCS: 2 ksi 
Drill-string Elements: 12 (10 DP + 1 (PDM Stator) + 1 (PDM Rotor/Bit)
 
Figure 15: Well-path for Precision 580 Corral Canyon 9-4 FED 122H Wellbore for 7.875” hole-section
Case 1: Pumps Off, Topdrive Rotary and Axial Motion on with ROP in Control Mode 
 
Figure 16: Comparison of RPM at Topdrive, drill-bit, mud-motor rotor and Stator Housing
Mud-pumps are off for the above mentioned Case 1. Rotate Mode Drilling, all attached charts in the first row governs the Topdrive parameters variation and the second row governs the dynamics of Bit/Rotor parameters assuming rotor is overlapped with bit and transfer its dynamics to bit directly
 
Figure 17: Surface and Downhole Parameters
ROP Control Mode for drilling is preferred here. WOB will adjust at Topdrive in accord with ROP.  Diff. pressure across motor will be accounted only when Pumps are on.
Case 2: Pumps On, Topdrive Rotary and Axial Motion on with ROP in Control Mode
Mud-pumps flow starts at t = 0 sec with 320 GPM Flow-rate and takes 10 seconds to reach full value, corresponding pumps RPM for 320 GPM flow-rate equals to 89.6.

 
Figure 18: Comparison of RPM at Topdrive, drill bit, Rotor and housing of Mud-motor
 
Figure 19: Surface and Downhole Parameters
Case 3: Slide + Rotate Mode, ROP in Control Mode, Pumps On
Slide Mode: From t = 0 sec to t = 350 sec, Pumps On, Topdrive Axial Motion On, Topdrive Rotary Off
Rotate Mode: From t = 350 sec onwards, with Topdrive RPM = 10, to counter the axial drag along the drill-string
 
Figure 20: Surface and Downhole Parameters
The peaks in chart above for downhole parameters highlights the transition from Slide mode to Rotary mode. In Slide mode, the drill-string above the stator starts to wind in counter clockwise direction due to reaction Torque applied from the rotor/bit and starts to build up gradually till Steady-state is achieved.
As the rotary at Topdrive starts, the drill string starts to unwind the residual stored torque suddenly and increases the DOC to a much higher value, resulting in high downhole reaction WOB and Torque.
 
Figure 21: Comparison of RPM at Topdrive, drill bit, Rotor and housing of Mud-motor

Similar effects can be seen in Bit RPM and Axial velocity. Once, the stored torque is released drill string will starts to twist in Clockwise sense and DOC starts to go down and achieve its expected value.
 
Figure 22: Comparison of Torque at different elements of drill-string to demonstrate the motor stall at Stator housing
The above chart shows the Torque variation at different Drill-string elements including the Topdrive and Rotor/Bit. It can be visualized from the chart that in the Slide mode, the Rotor is trying to twist the string in CW sense but the effect gets transferred on elements above Stator and act as counter twist in elements all the way to the Topdrive.
In Rotate mode, the peak highlights the sudden release of the negative stored torque and finally post transition, the torque starts to stabilize to expected values. The figure below shows the total rotation angle for Topdrive, Stator and Rotor/Bit elements.
 
Figure 23: Comparison of twist effects of rotary angle at stator housing of mud motor
Case 4: Rotate Mode, Pumps On, ROP Topdrive in Control Mode, Topdrive Rotary Step Function
 
Figure 24: Effect of different topdrive RPM of drum on Surface and Downhole parameters
In Figure 24, downhole parameters including the differential pressure at motor will change as the topdrive RPM of drum changes, this is primarily due to change in the DOC and reaction torque and WOB at bit which transfers the effects to the motor and vary its differential pressure.
 
Figure 25: Variation of Topdrive and Bit RPM due to change in Topdrive RPM of drum during drilling

 
Figure 26: Variation of Topdrive, motor input RPM and Total RPM at bit during drilling
 
Figure 27: Variation of Downhole parameters at bit and mud-motor differential pressure

Case 5: Rotate Mode, Pumps On/different Flow-rates, ROP in Control M., Topdrive Rotary Same
Pumps are on with different Flow-rates as input to Rotor element of Drill-string. The Topdrive Rotary is kept constant at 50 rpm and ROP Topdrive is set at 40 m/hr. with ROP Control Mode for drilling.
 
Figure 28: Variation of Topdrive and downhole parameters at different flow rates of mud-pumps

 
Figure 29: Variation of Topdrive, mud-motor input RPM and bit RPM Vs Time

Changing the mud-pumps flow rate, will change the input RPM at the rotor and this results in the change in total RPM at bit. DOC decreases and similar effects can be observed at downhole WOB, torque and motor different pressure. The Axial draw-works speed is kept constant during the simulation.
 
Figure 30: Variation of different downhole parameters at bit and mud-motor Vs Time

 
Figure 31: Variation of Topdrive and Bit Axial velocity vs Time
From the above charts, it is evident that the differential pressure across the motor is strongly dependent on the Topdrive parameters including RPM, Axial velocity, Pump Flow rates. Additionally, while on-bottom drilling, differential pressure across motor is dependent on the formation CCS, Depth of Cut of bit among other parameters.
Case 6: Pipe Rocking During Slide Mode, Pumps On, Topdrive ROP in Control Mode
Static friction is an important performance limiter when slide drilling with a downhole motor. Pipe rocking can be used as a low-cost technique to break the static friction in a section of the well and thereby reduce its negative effect. Pipe rocking simulation was used to find the rocking regime that maximizes the section of the string under conditions of dynamic friction, without losing tool-face control. 
When drilling directionally with a downhole motor, the drill pipe slides against the wellbore and encounters friction. This friction impairs force transfer to the bit, which reduces the rate of penetration (ROP) and also makes it difficult to achieve a desired drilling direction.  Additionally, poor tool-face control can translate into an increase in wellbore tortuosity, which in turn results in less efficient wellbore cleaning, increase in drag and a potential decrease in production rates in the future. Finally, poor hole cleaning and tortuosity may create restrictions whereby elastic energy gets stored in several sections of the drill-string, producing poor weight transfer and a risk of releasing bursts of energy against the formation. This can potentially damage bit and/or stall the motor. (Ref: URTEC-2019-1115-MS) 
 
Figure 32: Variation of Topdrive, mud-motor input RPM and bit RPM Vs Time during slide to rotate transition
Rocking a pipe consists in creating torsional oscillations from the top drive, rotating the string alternately forward and backwards. This action breaks the static friction in a segment of the string, providing better tool-face control and weight on bit (WOB) transfer during sliding operations.
Topdrive RPM is a sinusoidal function with Amplitude ±20 and cycle period 20 seconds. Mud-pump RPM kept the same as that in previous cases. The last subplot in above chart shows the Stator and Rotor/Bit RPM of the drill-string.
 
Figure 33: Variation of RPM at Topdrive and along different elements of drill-string
The chart above highlights the RPM variations at different drill-string elements including Topdrive, Stator and Rotor/Bit. Slide mode with no Topdrive rotary till 350 seconds. Afterwards, torsional oscillations of Topdrive initiated. An exploded in-depth view of the chart is presented below.
 
Figure 34: Variation of RPM at Topdrive and along different elements of drill-string (Exploded View)
The chart below highlights the Torque at different Drill-string Elements including the Stator and Rotor/Bit. The Magenta legend shows the Bit-torque clearly showing that the torsional oscillations at Topdrive were transferred to part of the drill-string and not all the way to the drill-bit. This in turn keeps the tool-face position intact based on the DOC and downhole reaction forces only. The borehole frictional forces along the axial direction gets reduced and in turn allowed better sliding motion for the drill-string.
 
Figure 35: Variation of Torque at Topdrive and along different elements of drill-string
The pipe rocking regime can be optimized by running several simulations and selecting the one that is found to be best for the operation. In general, rocking at higher RPMs can potentially lead to back-offs, as higher angular momentum in drilling tools can cause a joint to disconnect especially when suddenly changing direction of rotation. 
An optimum rocking RPM amplitude needs to be applied at Topdrive to have effective slide mode drilling. To have an in-depth look at the torque oscillations and RPM transfer along the drill-string length, an exploded view of the Torque data is presented below.
 
Figure 36: Variation of Torque at Topdrive and along different elements of drill-string (Exploded View)
The chart below highlights the total rotation angle for Topdrive and Stator element of the drill-string. As the Rotate mode is activated at topdrive, the counter twist stored in stator housing will gradually starts to release the trapped torque and approaches the 0 value before starts to increase in the positive Y direction down the path.
 
Figure 37: Variation of total Twist at Topdrive and Stator housing of mud-motor (Exploded View)
Case 7: Hard to soft rock transition during Slide Mode, Pumps On, Topdrive ROP in Control Mode
Well: Poker Lake Unit 30 BS 101H 9.875” hole-section
Max Well Inclination: 5^0
Drill-string Length: 1040 m.
Formation CCS: 12 ksi to 6 ksi transition in interbedded formation 
Drill-string Elements: 17 (9 DP + 3HWDP + 1 (PDM Stator) + 1 (PDM Rotor/Bit)
 
Figure 38: Surge in diff. pressure at motor is observed while drilling transition from hard to soft rock (field data)
 
Figure 39: Model validated to show motor diff. pressure surge while drilling transition from hard to soft rock
While drilling off in soft rock from hard rock, DOC spike is observed (Refer Figure: 38, 39). This is primarily because string will try to release the trapped energy while drilling the hard rock and results in increase in reaction torque and motor diff. pressure. The system resumes to previous state, as the Rock strength switched back to 12 ksi again from 6 ksi.

Note:

	Existing model parameter’s setting takes slightly lesser downhole WOB value at higher formation CCS values. Currently, at 60 ksi correspond to a Topdrive WOB of 45 kips. Expected value is slightly higher. Calibration/fine tuning is required in downhole WOB parameter constants (k_WOB) in model. Checked with MSE formulation (Ref: Teale 1965) with data observation for RPM = 120, ROP = 27 m/hr., at CCS = 10 ksi, WOB came of the order of 7981 lbs., from our Model WOB value at CCS = 10 ksi comes equal to 7500 lbs.)

	Include Mud Velocity into equation to properly assess the viscous drag due to fluid motion (Drilling Mud). Formulation already accounted for it, but for the time being, we have set the value of V_Mud=0 for all our test runs.

	We are calculating the differential pressure at the motor itself based on the Reaction Torque, and not allowing its effects transfer back up to surface. Additionally, “Fluid Compressibility” effect not accounted. The off-bottom SPP is not accounted for in model as for now. Effect of off-bottom SPP will be included in the model later by considering the flow in pipe to be approximately Laminar where, SPP∝ 〖Flow Rate〗^2.

 














Part-III: Drilling Simulator Features  
Drilling Simulator: Developed Features

Formulation of Coupled Axial-torsional Multi Elements Drill-string dynamics lumped parameter model has been documented. To assess the consistency of results across various Friction Models in future and also during modifications in existing model strategy, different sensitivity cases are tested and documented to account for numerical stability. These cases are of significant value to test the robustness of model under different Topdrive and Downhole conditions. 
They are categorized as follows:
	Number of Drill-string Elements (Segregation into Drill-pipes, HWDP, Collars and mud-motor elements)
	Flexibility to choose different Topdrive ROP and RPM Input function  (Constant, Ramp or Step)
	CCS of Formation (Numerically Stable solution for as high as 60000 Psi Rock Strength)
	Bit/Hole size and Number of Bit-blades (Symmetrical or Asymmetrical Blade positioning)
	Drill-pipe, HWDP and Collar size elements flexibility
	Tripping In/Out of hole, Drilling and Back-reaming operations
	Torque and Drag testing using side forces (Well-path inclination dependent)
	Tracking of Drilling Vs Tripping phase (Hole-depth Vs Bit Depth Positioning Tracker)
	Independent Rotary and Axial motion (SO/PU operations) 
	Pipe ID/OD and Tool joint size accounting and drill-string element’s length flexibility
	Inclusion and Exclusion capability of Noise effects while Drilling
	Selection flexibility for Open and Cased Hole Friction Factor 
	Stribeck Friction effect (Accounting of Exponential Non-linear Static to Dynamic Coulomb Friction Transition)
	Net frictional force accounting for borehole friction (Coulomb + Viscous)
	Well-bore inclination Angle (Well trajectory)
	Mud-Pumps Flow Rate (to feed Mud-motor input RPM)
	Slide and Rotate mode of drilling with Mud-motor (Flexibility of placing motor at different positions in BHA)
	Pipe Rocking phenomenon in Slide mode of drilling with Mud-motor 
	Motor Back-off Event testing capability in Slide Mode (with drilling depths having different Rock-strength)
	Low Friction Stabilizer and Lubrication trials testing capability

Number of Drill-string Elements: Segregate number of string elements as per the model flexibility and well path
ROP Topdrive Input Function: Ramp Up/down 
RPM Topdrive Input Function: Heaviside, Ramp Up/down
CCS of Formation: 2000 – 60000 Psi. Feeding the dependency of CCS on MD as model input 
Bit-size: 6”, 8.5”, 12.25”, 17.5” etc.
Number of Bit-blades: 3, 5, 7 and 9
Drill-pipe Size: 4”, 5”, 5.5” and 6.625” etc.
Tripping In/Out of hole: Code updated to account for tripping phenomena alongside drilling in model. 
Independent Rotary Motion: Topdrive Rotary motion is allowed with axial motion restricted.
Independent Axial Motion: Topdrive Axial motion is allowed with rotary motion restricted to account for significant string compression.
Pipe OD/ID: Accounted for different size (OD/ID) of Drill-string elements. Parameter dependency on Mass Moment of Inertia, area of cross-section, Polar Moment of Inertia.
Pipe Axial/Torsional Stiffness: Parameter dependency on pipe OD/ID and Length of string elements
Noise effect while Drilling: Incorporated noise in cutting mechanism by modifying CCS of formation to have realistic results with minimal vibration induced motion.
Stribeck Friction: Incorporated Stribeck Coulomb friction into model to account for swift transition between static to dynamic regimes for different drill-string Elements.
Wellbore Inclination: Added wellbore inclination into model to account for various well-paths for different wellbores and see the dynamics along the well trajectory.
Mud Velocity Effect: Added mud flow based viscous drag along the borehole to account for net frictional force comprises of Coulomb and Viscous forces and Torque coupled.
Mud-pumps Flow Rate: Incorporated mud-pumps flow rate into model to account for the RPM input to the Mud motor’s rotor element which feeds additional rpm to the drill bit in rotate mode. Fluid Compressibility is not included into model yet, part of future integration.
Mud Motor/Downhole Motor Integration: Accounted for downhole mud motor in drill-string to counter both Slide and Rotate mode of drilling using downhole motor.
Pipe Rocking Phenomenon in Slide mode: Added pipe-rocking phenomenon in model in Slide mode of drilling with downhole motor by providing to and fro motion to Topdrive rotary and break the additional drag along the axial direction of drill-string elements by keeping the tool-face intact
Motor back-off Events Testing: Drilling simulator possesses capability to test various cases of motor stall events.
Low Friction Stabilizers and Lubrication Trials: Lubrication trials and low friction stabilizers in BHA dynamics can be modeled using drilling simulator.
Drilling Simulator: Under Development Features

The current drilling simulator is robust to perform the basic drilling and tripping operations modeling in the presence/absence of downhole mud motor. Following key features development is under progress currently, to ramp up the usability of the tool for industry wide applications:
	Currently working to improve the computational performance speed and numerical stability of the model to simulate complex field events (e.g. harder rocks, additional motor stall events)

	Draw-works constraint is currently controlled ROP
	Need to upgrade to full multi-mode autodriller capability (WOB, ROP, Torque, Diff. Pressure with adjustable gain parameters)
	Working along with Schlumberger for they use iDrill to model their Omron and Cameron autodrillers with DrillOps 
	Need to integrate SoftSpeed, SoftTorque, Z-Torque control models and tuning for these cases is required (University of Calgary has this modeled)

	Top drive constraint is constant RPM
	Need to model topdrive stalling and interaction with auto driller torque limits
	Currently, we have guidance based on limited field operations (Qatar, Hebron and land rigs), need to improve this capability inclusion
	Need to integrate SoftSpeed, SoftTorque, Z-Torque control models and tuning for these cases is required (University of Calgary has this modeled)

	Working with Nabors to put their Autodriller on top of our Drilling Simulator to help improve the performance of the existing capabilities

	Extension of the effort to Stiff-string modeling is under development by Dr. Gregory Payette, to leverage the existing capability models and create another fit-to-purpose tool

	Another opportunity for improving the drilling simulator is the inclusion of automation optimization engines (Schlumberger’s DrillOps, Pason’s AutoDAS, Nabors Drill Smart and H&P’s Auto Slide systems) 

	Current drilling simulator bit-rock interaction model is complex and accounts for the tracking of the rock surface cutting dynamics. We are reviewing additional DOC based models to make the modeling simpler without jeopardizing the capability of the tool

	Other potential upgrades we are focusing on includes: robust borehole friction accounting, angle, spiral and reverse whirl phenomenon. Hydraulics and pump off force with inclusion of pressure pulse effects 













Appendix  
Drilling Simulator: Code Functionality

The Drilling simulator capabilities can be utilized to model various drilling and tripping scenarios and assess the severity of various drilling dysfunctions including stick-slip. 
User needs to provide data for various input parameters to model the dynamics of different studies. Some of the key outline parameters are:
	Well Survey data for the desired wellbore: Primarily MD, inclination angle, Azimuthal Angle, TVD and VS data. Correct MD and inclination angle values will be of significant help to account the correct borehole friction variation along the length of the drill-string while tuning the model. 

	Different Hole-sections Actual Tops and Section TD data: Different Hole-sections Actual Tops and section TD data is required to assess which section drilling analysis needs to performed and range of simulation runs corresponding to the data.

	BHA specs sheet of the hole-section under consideration: Specifications primarily consist of all the elements of BHA along with the following data for individual elements:
	Nominal OD and ID information
	Length of individual elements of BHA
	Adjusted weights of individual elements of BHA
	Element material specs (If not available we can assume Steel for all the components to assess the E and G parameters for calculation of Element’s stiffness)

	Formation Tops and Rock Strength: Formation Tops and approximated Rock-strength data for the section under consideration.

	Drilling Mud type: Type of drilling mud (NAF or WBM) will help us to assess the static and dynamic Coulomb friction coefficient limits. Additionally, viscous friction dampers value will also depend on the type of mud used.

	Actual T&D Charts for the hole-sections: SO/PU/Rot and FF values correspondingly and Actual T&D Charts to help user tune in the FF input values in model

	Surface Drilling Mechanics Data Frequency: 1 Sec data frequency preferred 

	Downhole Drilling Mechanics Data: OPTI-drill data, MWD data (If Available)

	Tripping Operations accounting in Model: Trip length parameter in the code accounts for the tripping operations while running various simulation runs. Setting the right Trip length will help the user to avoid the bit-rock interaction forces during the tripping run. Bit-rock interaction forces will be active for the duration when bit is on bottom.

	Adaptive Time step size for Model: Time step size for the model will be dependent on several parameters primarily being number of drill-string elements and rock strength under consideration. Adjust the value to refined size to avoid numerical instability in the solution.
Data Validation steps:
	Torque and Drag FF Assessment: The first step for setting up the model after getting all the input parameters is to assess the On-bottom and Off-bottom TAD simulation runs. This will provide us with the approximate FF for various drill-string Elements at multiple Survey points. The details for performing the T&D assessment simulation can be categorized into:

	Off-bottom T&D Assessment during Tripping: The off-bottom T&D simulation run includes tripping operation followed by extracting the FF values at different string elements at various Survey points by applying a pre-defined axial and rotary Topdrive input conditions based on the field data provided. The model input coefficient of static and dynamic Coulomb friction values are set to 0.25 and 0.2, respectively and calculations are made w.r.t. these values. To account for the Stribeck friction, make sound judgement on the range of variation of static and dynamic FF values
	With Topdrive Rotary Off
	With Topdrive Rotary On

	On-bottom T&D Assessment during Drilling: In this case, simulation run is performed during the drilling phase to better assess the effect of (Rock-strength + Torque Response) on the FF Vs the effect of borehole friction on string Elements assessed during the Off-bottom case mentioned above.

Note: All the steps mentioned above are general procedure for wells data validation before progressing to run the drilling simulations.

	Tripping and Drilling Data Validation: Tripping and Drilling Data validation for NFQ18-05 17.5” hole-sections as per field data provided. Adjust the friction factors to match the lower torque seen deeper in the well. (Variance in friction factor to see the matched data)

	Stick-slip detection: Assessment of Stick-slip indication while drilling the hole-section using drilling simulator will primarily be depending on the Topdrive Axial and Rotary input conditions, Rock-strength and the static and dynamic FF parameters variations.
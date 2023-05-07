Download Link: https://assignmentchef.com/product/solved-esc794-homework-2
<br>
Suppose a constant disturbance <em>δ </em>is assumed for the prediction horizon, resulting in the single-output model

<table width="226">

 <tbody>

  <tr>

   <td width="70"><em>x</em>(<em>k </em>+ 1)</td>

   <td width="25">=</td>

   <td width="130"><em>A<sub>d</sub>x</em>(<em>k</em>) + <em>B<sub>d</sub>u</em>(<em>k</em>)</td>

  </tr>

  <tr>

   <td width="70"><em>y</em>(<em>k</em>)</td>

   <td width="25">=</td>

   <td width="130"><em>Cx</em>(<em>k</em>) + <em>Du</em>(<em>k</em>) + <em>δ</em></td>

  </tr>

 </tbody>

</table>

<ol>

 <li>Obtain the pertinent prediction equations and show that the optimization cost function becomes</li>

</ol>

where ∆ is a column vector with all elements equal to <em>δ</em>.

<ol>

 <li>For the nonlinear plant</li>

</ol>

<table width="147">

 <tbody>

  <tr>

   <td width="32"></td>

   <td width="25">=</td>

   <td width="90">sin(<em>x</em><sub>1</sub>)<em>x</em><sub>2 </sub>+ <em>u</em></td>

  </tr>

  <tr>

   <td width="32"></td>

   <td width="25">=</td>

   <td width="90">cos(<em>x</em><sub>1</sub>)<em>x</em><sub>1</sub></td>

  </tr>

 </tbody>

</table>

consider the problem of regulation to the origin under the constraint |<em>u</em>| ≤ 1<em>/</em>2 and unit box constraints for the states. Use an MPC linearization approach with linearized model updates at each control iteration (find Jacobians analytically). Take <em>λ </em>= 2, <em>n<sub>y </sub></em>= 5, and <em>n<sub>u </sub></em>= 4.

Complete the provided code templates to run MPC simulations with and without disturbance compensation. Show summary plots with the predicted and closed-loop outputs, with and without disturbance compensation. Re-tune the disturbance compensation MPC to recover or exceed the performance of the first case. Note that some settings may result in unfeasible OPCs at some iterations.

<strong>2 (mini-project) </strong>Consider the elevator control problem discussed in class. Your task is to complete the provided code template to simulate closed-loop system operation with linear predictions, as discussed in class. The objective is to move the elevator to a target position using an optimal combination of motor power and motion performance, while maintaining constraints on the cable tension, and car acceleration and velocity for passenger comfort. The mathematical model in C-T can be derived as

where <em>V </em>is the control voltage and the parameters are as follows:

<table width="183">

 <tbody>

  <tr>

   <td width="76">Parameter</td>

   <td width="58">Units</td>

   <td width="48">Value</td>

  </tr>

  <tr>

   <td width="76"><em>J<sub>t</sub></em></td>

   <td width="58">kg-m<sup>2</sup></td>

   <td width="48">20</td>

  </tr>

  <tr>

   <td width="76"><em>a</em></td>

   <td width="58">N-m/A</td>

   <td width="48">12</td>

  </tr>

  <tr>

   <td width="76"><em>R<sub>m</sub></em></td>

   <td width="58">Ω</td>

   <td width="48">0.1</td>

  </tr>

  <tr>

   <td width="76"><em>R</em></td>

   <td width="58">m</td>

   <td width="48">1</td>

  </tr>

  <tr>

   <td width="76"><em>m<sub>c</sub></em></td>

   <td width="58">kg</td>

   <td width="48">1000</td>

  </tr>

  <tr>

   <td width="76"><em>m<sub>w</sub></em></td>

   <td width="58">kg</td>

   <td width="48">750</td>

  </tr>

 </tbody>

</table>

The tensions on the car and counterweight cables are given by

<em>T<sub>c </sub></em>= <em>m<sub>c</sub></em>(<em>y</em>¨+ <em>g</em>)

<em>T<sub>w </sub></em>= <em>m<sub>w</sub></em>(<em>g </em>− <em>y</em>¨)

Assume the elevator starts at <em>y</em><sub>0 </sub>= 0 and must stop at <em>y </em>= <em>y<sub>f </sub></em>in a reasonable time. Motor voltage must be within ±40 V and the elevator speed and acceleration must be limited by ±3 m/s and ±0<em>.</em>2<em>g</em>.

The current consumed by the motor is given by

Therefore the power <em>P </em>= <em>V i<sub>m </sub></em>is a quadratic function whose integral can be considered for minimization over some horizon. The position control objective can be measured as an integral quadratic error, using <em>e </em>= <em>y </em>− <em>y<sub>f</sub></em>.

Approach: Discretize the plant and use an incremental formulation. To eliminate gravity bias, define

To limit control bandwidth, we impose upper and lower bounds on ∆<em>u </em>of ±1000 V/s and also include a weight on the running cost to penalize ∆<em>u</em>. The running cost will therefore have the form

<em>e</em><sup>2 </sup>+ <em>λP </em>+ <em>λ<sub>u</sub></em>(∆<em>u</em>)<sup>2</sup>

<ol>

 <li>Using <em>u </em>as new control, obtain a continuous-time state-space representation where the states are position and velocity. Discretize it with ZOH, with <em>T<sub>s </sub></em>= 0<em>.</em>05 s. Verify that the discrete states have the same physical meaning as their continuous counterparts.</li>

 <li>Use the incremental formulation and find the augmented matrices <em>A<sub>d </sub></em>and <em>B<sub>d</sub></em>. Write code to find matrices <em>H<sub>pos</sub></em>, <em>P<sub>pos</sub></em>, <em>H<sub>vel </sub></em>and <em>P<sub>vel </sub></em>associated with position and velocity predictions. Treat the extended state variable <em>u </em>as an output and write code to find its corresponding prediction matrices <em>H<sub>u </sub></em>and <em>P<sub>u</sub></em>, all this for any <em>n<sub>y</sub>,n<sub>u</sub></em>.</li>

 <li>Show that the cost function becomes</li>

</ol>

where

<ul>

 <li>= 2(<em>H</em><em>posT H</em><em>pos </em>+ <em>λR</em>2 <em>m</em>2<em>H</em><em>uTH</em><em>u </em>− <em>λ</em>2<em>H</em><em>uTH</em><em>vel </em>+ <em>λ</em><em>uI</em>) <em>a R R</em></li>

 <li><em>T T                  </em>2<em>λR</em><em>m      T                  T</em></li>

</ul>

<em>f </em>= 2(−<em>y</em>ˆ<em>f </em>+ <em>x</em><em>a P</em><em>pos</em>)<em>H</em><em>pos </em>+ 2 2 <em>x</em><em>a P</em><em>u H</em><em>u</em>+ <em>a R</em>

2<em>λm</em><em>cwgR</em><em>m                      λ     </em><em>T         T                          T</em>

<em>a</em>2      <em>H</em><em>u </em>− <em>R</em>2<em>x</em><em>a </em>(<em>P</em><em>u H</em><em>vel </em>+ <em>P</em><em>velH</em><em>u</em>) − <em>λm</em><em>cwgH</em><em>vel</em>

where <em>x<sub>a </sub></em>is the augmented state feedback used in the solution of the OCP.

<ol>

 <li>The acceleration constraints are more stringent than the cable tension constraints, so only the former need to be considered. Find the <em>C </em>and <em>D </em>matrices for voltage <em>V </em>(treat as an output which depends on the extended state <em>u</em>), velocity and acceleration. Use that information to write code that builds the pertinent <em>H<sub>c </sub></em> Also find <em>C<sub>c </sub></em>based on the desired constraints for ∆<em>u</em>. Build all pertinent vectors and matrices to complete the data for the quadratic program.</li>

 <li>Complete the provided code template to obtain closed-loop simulations.</li>

 <li>For a 10-floor move (30 m), the elevator should not overshoot/undershoot by more than 8 cm and should make the trip in less than 12 seconds.</li>

 <li>Tune the system for minimum energy consumption considering an up-down 30 m sequence of moves. Do some research on the actual power consumption of elevators. Note that the model incurs energy consumption just to hold the elevator still. You should not calculate energy consumption after the target floor has been reached.</li>

 <li>Document all aspects of your solution to this project. You must submit running code, with comments, as a single m-file.</li>

</ol>
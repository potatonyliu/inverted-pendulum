# Table of Contents

# Symbolic Conventions

$$ \begin{align*} g&\approx 9.81 m/s^2\\ m&=\text{mass of the pendulum (bob)}\\ M&=\text{mass of the cart}\\ r&=\text{length of the pendulum}\\ t&=\text{time}\\ x_p&=\text{x-cordinate of the pendulum}\\ y_p&=\text{y-cordinate of the pendulum}\\ F&= \text{force applied on the cart, only x-component} \\\theta&=\text{radian of the pendulum from positive x axis, function of t}\\ \phi&=\text{radian of the pendulum from positive y-axis, function of t}\\ x&=\text{position of the cart, function of t}\\ \dot \theta / \dot \phi&=\text{angular velocity of the pendulum, first derivative of } \theta / \phi\\ \dot x&=\text{velocity of the cart, first derivative of }x\\ \ddot \theta / \ddot \phi&=\text{angular accerlation of the pendulum, second derivative of } \theta / \phi\\ \ddot x&=\text{accerlation of the cart, second derivative of }x\\ T&=\text{Kinetic energy}\\ V&=\text{Potential energy} \end{align*} $$

*”mg” should be “-mg”

# Important Results

## LQR Parameters

$$ \begin{bmatrix} x\\ \dot x\\ \phi\\ \dot \phi\\ \end{bmatrix}\begin{bmatrix} z_1\\ z_2\\ z_3\\ z_4\\ \end{bmatrix}$$

where:

$$ \begin{align*} \dot z_1 &= z_2\\ \dot z_2 &= \frac{gm\phi+F}{M}\\ \dot z_3 &= z_4\\ \dot z_4 &= \frac{F+(M+m)g\phi}{rM} \end{align*} $$

$$ \begin{align} A&=\begin{bmatrix} 0 & 1 & 0 & 0\\ 0 & 0 & \frac{gm}{M} & 0\\ 0 & 0 & 0 & 1\\ 0 & 0 & \frac{(M+m)g}{rM} & 0\\ \end{bmatrix} \\ B&=\begin{bmatrix} 0\\ \frac{1}{M}\\ 0\\ \frac{1}{rM}\\ \end{bmatrix} \\ Q &= \text{diag}(10,1,10,1) \\R &=1 \end{align} $$

$$ K=lqr(A,B,Q,R) $$

Where K is the gain for force applied given the state. A and B are the linearized terms for the state variable we just obtained; Q is the weight of penalty for error (arbitrarily set at 10 times more for position & angle than velocity); R is just a scalar to control the force. I don’t fully understand lqr yet, I will need a bit more research on this, but it does give me the correct K for small $\phi$ right now.

## Non-linear Second Order Differential Equations for $\ddot \phi$ and $\ddot x$

$$ \begin{align} \ddot x&=\frac{gm\sin(\phi)\cos(\phi)+F-mr\sin(\phi)\dot \phi^2}{(M+m)-m\cos^2(\phi)} \\ \ddot \phi &= \frac{\cos(\phi) F-mr \cos(\phi)\sin(\phi)\dot \phi^2+(M+m)g\sin(\phi)}{r((M+m)-m\cos^2(\phi))} \end{align} $$

Note that I canceled $\cos(\phi)$ from the first equation to prevent division by 0 when system is at equilibrium.

## MATLAB Simulation for Different Starting $\phi$

Simulated with:

```matlab
M = 1.0;
m = 0.2;
r = 1.0;

Q = diag([10,1,10,1]);
R = 1;

tspan = [0 7];
```



| Angle (rad) | Angle (degree) | Max Force (N) | Max Velocity (m/s) |
| ----------- | -------------- | ------------- | ------------------ |
| 0.1         | 5.7296         | 4.6824        | 0.60586            |
| 0.2         | 11.459         | 9.3647        | 1.2251             |
| 0.3         | 17.189         | 14.047        | 1.8747             |
| 0.4         | 22.918         | 18.729        | 2.5725             |
| 0.5         | 28.648         | 23.412        | 3.3495             |
| 0.6         | 34.377         | 28.094        | 4.2145             |
| 0.7         | 40.107         | 32.777        | 5.2448             |
| 0.8         | 45.837         | 37.459        | 6.4942             |
| 0.9         | 51.566         | 42.141        | 8.056              |

# Equations

Started from the [Euler-Lagrange equation](https://youtu.be/3apIZCpmdls):

$$ \frac{\partial L}{\partial q_i} - \frac{d}{dt} \left( \frac{\partial L}{\partial \dot{q}_i} \right) = 0 $$

We can have 2 of these equations independent of each other for x and $\theta$.

$$ \begin{cases} \frac{\partial L}{\partial x} - \frac{d}{dt} \left( \frac{\partial L}{\partial \dot{x}} \right) = F\tag{1}\\ \frac{\partial L}{\partial \theta} - \frac{d}{dt} \left( \frac{\partial L}{\partial \dot{\theta}} \right) = 0 \end{cases} $$

Find the Lagrangian with kinetic and potential energy:

$$ L=T-V=\frac{m}{2}(\dot x ^2-2\dot xr \sin (\theta)\dot\theta + r^2\dot \theta^2)+ \frac{1}{2}M\dot x ^2 - mgr(\sin \theta+1) $$

Plug the Lagrangian into both equations in (1) to obtain the 2 key second order differential equations:

$$\begin{cases} F=(M+m)\ddot x - mr\cos(\theta)\dot \theta^2 - mr\sin(\theta)\ddot \theta\\ \ddot x\sin(\theta)-r\ddot \theta = g\cos(\theta) \end{cases}$$

$$\ddot x = \frac{F + mr\cos(\theta)\dot \theta^2 + mr\sin(\theta)\ddot \theta}{M+m}=\frac{g\cos(\theta)+r\ddot \theta}{\sin (\theta)}$$

Express $\ddot x$ and $\ddot \theta$ in in first order terms:

$$ \ddot \theta = \frac{sin\theta F+\frac{1}{2}mr \sin(2\theta)\dot \theta^2-(M+m)g\cos \theta}{r((M+m)-m\sin^2\theta)}\\ \ddot x = \frac{g\cos(\theta)+\frac{sin\theta F+\frac{1}{2}mr \sin(2\theta)\dot \theta^2-(M+m)g\cos \theta}{(M+m)-m\sin^2\theta}}{\sin (\theta)} \\ \ddot x=\frac{-gm\cos\theta\sin^2\theta+\sin\theta F+\frac{mr}{2}\sin(2\theta)\dot \theta^2}{\sin\theta((M+m)-m\sin^2\theta)}$$

$$\ddot x=\frac{-gm\cos\theta\sin^2\theta+\sin\theta F+\frac{mr}{2}\sin(2\theta)\dot \theta^2}{\sin\theta((M+m)-m\sin^2\theta)} \\ \ddot \theta = \frac{sin\theta F+\frac{1}{2}mr \sin(2\theta)\dot \theta^2-(M+m)g\cos \theta}{r((M+m)-m\sin^2\theta)} $$

$$ \ddot x=\frac{gm\sin(\phi)\cos(\phi)+F-mr\sin(\phi)\dot \phi^2}{(M+m)-m\cos^2(\phi)} \\ \ddot \phi = \frac{\cos(\phi) F-mr \cos(\phi)\sin(\phi)\dot \phi^2+(M+m)g\sin(\phi)}{r((M+m)-m\cos^2(\phi))} $$

### Linear Approximation

sin(π/2 + φ) ≈ 1

cos(π/2 + φ) ≈ -φ

sin²(π/2 + φ) ≈ 1

For small φ: φ² ≈ 0 (neglect)

$$ F=rM\ddot \theta-(M+m)g\phi=M\ddot x - gm \phi $$

$$ \begin{bmatrix} x\\ \dot x\\ \phi\\ \dot \phi\\ \end{bmatrix}\begin{bmatrix} z_1\\ z_2\\ z_3\\ z_4\\ \end{bmatrix}$$

where:

$$ \begin{align*} \dot z_1 &= z_2\\ \dot z_2 &= \frac{gm\phi+F}{M}\\ \dot z_3 &= z_4\\ \dot z_4 &= \frac{F+(M+m)g\phi}{rM} \end{align*} $$

$$ A=\begin{bmatrix} 0 & 1 & 0 & 0\\ 0 & 0 & \frac{gm}{M} & 0\\ 0 & 0 & 0 & 1\\ 0 & 0 & \frac{(M+m)g}{rM} & 0\\ \end{bmatrix} $$

$$ B=\begin{bmatrix} 0\\ \frac{1}{M}\\ 0\\ \frac{1}{rM}\\ \end{bmatrix} $$

## With Rod

$$ m(r\ddot x \sin{\theta}-r^2 \ddot \theta)-mgr\cos \theta + \frac{M_r}{2}(r\ddot x sin \theta - \frac{2}{3}r^2\ddot \theta)-M_r gr_0 \cos \theta = 0 $$

$$ F= m(\ddot x - r\cos \theta \cdot \dot \theta^2 - r \sin \theta \cdot \ddot \theta) + \frac{M_r}{2}(2\ddot x - r \cos \theta \cdot \dot \theta ^2 - r \sin \theta \cdot \ddot \theta) + M \ddot x $$

------

$$ \ddot x = \frac{(m+\frac{M_r}{2})(r\cos\theta \cdot \dot \theta ^2+r\sin \theta \cdot \ddot \theta)+F}{m+M_r+M} $$

$$ \ddot \theta = \frac{mr(\ddot x \sin \theta - g \cos \theta)+\frac{M_r}{2}r \ddot x \sin \theta - M_r gr_0 \cos \theta}{\frac{M_r r^2}{3}+mr^2} $$

------

# Discussion

Intuitively yes. LQR calculates a cost function based on how much our states different from 0 + how much force is used. We can adjust the weight of each term to match what we want (perhaps error in angle matters much more than error in cart velocity). We adjust the speed (correction of error) based on the magnitude of the error. Which would overshoot because we can only accelerate so much and we want it to converge quickly instead of taking 5000 years before it reaches equilibrium.

------

Ooo, dang, it looks like our worlds are startling to overlap…

In the pic below, don’t the x(t) and the y(t) graphs start to resemble the MATLAB output for different starting values of $\phi$

Does it make sense to start thinking about the job of the LQR algorithm as something that converts the natural tendency of the inverted pendulum (as an unstable **saddle** that wants to fall away from $\phi = 0$ to its natural (gravity induced) resting position at $\phi = \pi$ or $\phi = 180°$ into a **spiral sink** that overshoots $\phi = 0$ by less and less, spiraling around the solution until we are effectively balanced?



Thank you for your feedback Dan!

1. Yes, g should be negative.
2. I described them as functions of t so that it is clear that $\frac{d}{dt} \theta$ is meaningfully its derivative $\dot \theta$. They are dependent on time in the sense that given the initial condition, we have a function that can take t and return a unique $\theta$ at that moment. Yes. Each $\theta$ depends on its previous state, that doesn’t mean we don’t have a function that maps t to $\theta$, although we don’t have it in close form.
   1. Yeah! I think we’re talking about the same idea. I’ll look through the next bit of your post when I have a bit of time to do so. -d
   2. Actually @Daniel, I used -mgh in my calculation, g is positive, negative sign was included.
   3. Great! I haven’t had a moment yet to look through the equations, but I will!

------

Hi @Tony Liu !

Thanks for starting this Symbolic Conventions doc! It is already very helpful. Two thoughts:

1. I believe our gravity constant should be negative so as to point it in the right direction.
2. I’m looking at your definitions for $\theta$ and $\phi$ where you suggest that they are a function of t. Now that I am an expert a differential equations :) I’m hesitant to describe them in that way. To me this suggests that that for any given t (as the independent variable), we can ask this function for the state of $\theta$ and $\phi$, but I’m not convinced that is true.

I’m wondering if it’s proper to consider those variables as an autonomous differential equation in the form:

$\frac{d\phi}{dt} = f(\phi)$

or

$\frac{d\theta}{dt} = f(\theta)$

suggesting that the state of $\theta$ is dependent on the previous state of $\theta$ rather than t.

Let me know your thoughts!

ps… apparently you can’t do Latex in a comment, so I’m crashing in on the body!
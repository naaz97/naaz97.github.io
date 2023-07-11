---
title: Robotics:Perception -- Study Notes - Week 1
categories: Robotics
---

>This course is offered by coursera. You can audit the course for free. Check it out [here](https://www.coursera.org/learn/robotics-perception/)

## Perspective projection

* **Vanishing point** - The vanishing point is where parallel lines receding away from you would converge on the horizon line.
* **Horizon** - The plane on the ground if we were receding to infinity, if there is no building behind this we can see the whole earth infront us. This is where the ground plane will intersect at infinity. 

<center>
<img src="https://github.com/naaz97/naaz97.github.io/blob/main/source/_posts/robotics:perception/image-1.png?raw=true"  width="30%" height="30%">
</center>

### The projective plane

<center>
<img src="https://github.com/naaz97/naaz97.github.io/blob/main/source/_posts/robotics:perception/image-2.png?raw=true"  width="30%" height="30%">
</center>

- Homogeneous coordinates represents coordinates in 2 dimensions in 3 dimensional vector.

<center>
{% katex %}
\begin{bmatrix}
x \\
y  
\end{bmatrix} \longrightarrow
\begin{bmatrix}
x \\
y \\
1
\end{bmatrix}
{% endkatex %}
</center>

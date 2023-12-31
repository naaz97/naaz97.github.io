---
title: Robotics:Perception:Study Notes - Week 1
tags: 
- Robotics
- Perception
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
<img src="https://github.com/naaz97/naaz97.github.io/blob/main/source/_posts/robotics:perception/image-homo-coordinates.png?raw=true"  width="50%" height="30%">
</center>

- Point 
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

- Line
    - A line is a plane of rays through the origin
    <center>
    <img src="https://github.com/naaz97/naaz97.github.io/blob/main/source/_posts/robotics:perception/image-2.png?raw=true"  width="50%" height="30%">
    </center>
     
     - All rays from the origin satisfy: {% katex %} \begin{aligned} 0 & = ax + by + cz \end{aligned} {% endkatex %}
     
     <center>
        {% katex %}
        \begin{bmatrix}a&b&c\end{bmatrix} 
        \begin{bmatrix}
        x \\
        y \\
        z
        \end{bmatrix} = 0
        {% endkatex %}
    </center>

    <center>
    <img src="https://github.com/naaz97/naaz97.github.io/blob/main/source/_posts/robotics:perception/image-3.png?raw=true"  width="50%" height="30%">
    </center>

    <center>
    <img src="https://github.com/naaz97/naaz97.github.io/blob/main/source/_posts/robotics:perception/image-4.png?raw=true"  width="50%" height="30%">
    </center>

- Projective line from two points

    <center>
    <img src="https://github.com/naaz97/naaz97.github.io/blob/main/source/_posts/robotics:perception/image-5.png?raw=true"  width="50%" height="30%">
    </center>
w = 0.18#0.2 #0.25
h = 0.1#0.12
glMaterialfv(GL_FRONT_AND_BACK,  GL_AMBIENT_AND_DIFFUSE, [0,0,1,1])
glMaterialfv(GL_FRONT_AND_BACK,  GL_SPECULAR           , [1,1,1,1])
glMaterialfv(GL_FRONT_AND_BACK,  GL_EMISSION           , [0,0,1,1])
glMaterialfv(GL_FRONT_AND_BACK,  GL_SHININESS          , 10)
glRectd(-w/2., -h/2., w/2., h/2.)

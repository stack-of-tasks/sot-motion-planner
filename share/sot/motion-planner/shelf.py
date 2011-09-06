w = 0.787
h = 0.391
glMaterialfv(GL_FRONT_AND_BACK,  GL_AMBIENT_AND_DIFFUSE, [0,0,1,1])
glMaterialfv(GL_FRONT_AND_BACK,  GL_SPECULAR           , [1,1,1,1])
glMaterialfv(GL_FRONT_AND_BACK,  GL_EMISSION           , [0,0,1,1])
glMaterialfv(GL_FRONT_AND_BACK,  GL_SHININESS          , 10)
glRectd(-w/2., -h/2., w/2., h/2.)
glTranslatef(0.,0.,.5)
glRectd(-w/2., -h/2., w/2., h/2.)

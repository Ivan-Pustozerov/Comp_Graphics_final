from PIL import Image, ImageOps;
from math import *;
from random import randint;

import numpy as np;
import time;
import sys;

class Quaternion():
    def __init__(self,a,b,c,d):
        self.a = float(a);
        self.b = float(b);
        self.c = float(c);
        self.d = float(d);

    def multyply(self, other):
        a1 = self.a;
        b1 = self.b;
        c1 = self.c;
        d1 = self.d;
        
        a2 = other.a;
        b2 = other.b;
        c2 = other.c;
        d2 = other.d;
        
        a = (a1*a2 - b1*b2 - c1*c2 - d1*d2);
        b = (a1*b2 + b1*a2 + c1*d2 - d1*c2);
        c = (a1*c2 - b1*d2 + c1*a2 + d1*b2);
        d = (a1*d2 + b1*c2 - c1*b2 + d1*a2);
        
        return Quaternion(a, b, c, d);
        
    def add(self, other):
        a1 = self.a;
        b1 = self.b;
        c1 = self.c;
        d1 = self.d;
        
        a2 = other.a;
        b2 = other.b;
        c2 = other.c;
        d2 = other.d;
        
        a = a1+a2;
        b = b1+b2;
        c = c1+c2;
        d = d1+d2;
        
        return Quaternion(a, b, c, d);
        
    def opposite(self):
        a = self.a;
        b = -self.b;
        c = -self.c;
        d = -self.d;
        
        return Quaternion(a, b, c, d);
        
    def norma(self):
        a = self.a;
        b = self.b;
        c = self.c;
        d = self.d;
        
        return sqrt(a*a + b*b + c*c + d*d);

    def getMatrix(self):
        
        a = self.a;
        b = self.b;
        c = self.c;
        d = self.d;
        
        a11 = a*a + b*b - c*c - d*d;
        a12 = 2*b*c - 2*a*d;
        a13 = 2*b*d + 2*a*c;
        
        a21 = 2*b*c + 2*a*d;
        a22 = a*a - b*b + c*c - d*d;
        a23 = 2*c*d - 2*a*b;
        
        a31 = 2*b*d - 2*a*c;
        a32 = 2*c*d + 2*a*b;
        a33 = a*a - b*b - c*c + d*d;
        
        return np.array([[a11,a12,a13],
                         [a21,a22,a23],
                         [a31,a32,a33]]);

class Model():
    def __init__(self, shift, angle_st, Filename, TextureFile, scale):
        self.Filename=Filename;
        self.angStart=angle_st;
        self.shift=shift;
        self.verts=[];
        self.faces=[];
        self.faceTextures = [];#координаты
        self.textureMap ={};#номера текстур
        self.texture = np.array(Image.open(TextureFile));#текстура - цвета
        self.scale = scale;
        
    def readObj(self):
        verts=[];
        with open(self.Filename, 'r') as file:
            for line in file:
                parts=line.strip().split();
                if not parts:
                    continue;
                #Вершина:
                if parts[0]=='v':
                    #///добавить больше 3 вершин - поддержку!!!
                    verts.append([float(parts[1]),float(parts[2]),float(parts[3])]);
                    
                elif parts[0]=='vt':
                    x = float(parts[1]);
                    y = float(parts[2]);
                    self.faceTextures.append((x,y));
                    
                elif parts[0]=='f':
                    
                    dots=[];
                    textureDot=[];
                    for i in range(1,len(parts)):
                        dots.append(int(parts[i].split('/')[0]));
                        textureDot.append(int(parts[i].split('/')[1]));
                        
                    #опорная - 1 точка:
                    Mdot=dots[0];
                    Mtexture = textureDot[0];
                    
                    
                    for i in range(1,len(dots)-1):    
                        dot2=dots[i];
                        dot3=dots[i+1];
                        texture2 = textureDot[i];
                        texture3 = textureDot[i+1];
                        self.faces.append([Mdot,dot2,dot3]);
                        self.textureMap[(Mdot,dot2,dot3)] = (Mtexture, texture2, texture3);
                    
        self.verts=np.array(verts);
        print("end_parsing");
    
    def RotateBase(self, ang_arr):
        a=ang_arr[0];
        b=ang_arr[1];
        z=ang_arr[2];
        self.angle=ang_arr;
        
        Rx=np.array(([1,0,0],[0,cos(a),sin(a)],[0,-sin(a),cos(a)]));           
        Ry=np.array(([cos(b),0,sin(b)],[0,1,0],[-sin(b),0,cos(b)]));
        Rz=np.array(([cos(z),sin(z),0],[-sin(z),cos(z),0],[0,0,1]));
                                                                               
        R = Rx@Ry@Rz;
        
        self.verts = (R @ self.verts.T).T;
     
    def RotateQuaternion(self, ang_arr):
        a=ang_arr[0];
        b=ang_arr[1];
        z=ang_arr[2];
        self.angle = ang_arr;
        
        qx = Quaternion(cos(a/2), sin(a/2), 0, 0);
        qy = Quaternion(cos(b/2), 0, sin(b/2), 0);
        qz = Quaternion(cos(z/2), 0, 0, sin(z/2));
        
        q = qx.multyply(qy).multyply(qz);
        R = q.getMatrix();
        
        self.verts = (R @ self.verts.T).T;
            
    def RotateAddANG(self, addx, addy, addz):
        return self.Rotate(self.angle[0]+ang_arr[0], self.angle[1]+ang_arr[1], self.angle[2]+ang_arr[2]);  

    def Shift(self, shift):
        shift=np.array(shift);
        for i in range(len(self.verts)):
            self.verts[i]+=shift;
        self.shift=shift;   
        
    def ShiftAdd(self, addshift):
        return self.Shift(shift+np.array(addshift));
    
    def getVerts(self):
        return self.verts;
    
    def getFaces(self):
        return self.faces;
    
    def getFaceTextures(self):
        return self.faceTextures;
    
    def getTextureMap(self):
        return self.textureMap;
    
    def getTexture(self):
        return self.texture;
    
    def getScale(self):
        return self.scale;


class Display():
    def __init__(self, Filename="default", height=1000, width=1000, matrix=0):
        self.File=Filename;
        self.w=width;
        self.h=height;
        self.zBuffer=np.full((self.h,self.w),float('inf'));
        if(matrix==0):
            self.screen=np.zeros((self.h,self.w,3),dtype=np.uint8);
        else:
            self.screen=np.array(matrix);
            
    def drawModel(self, model:Model, isSmooth, isTexture, screenShift = 0, mode="net"):
        
        if(screenShift == 0):
            screenShift=[self.w/2,self.h/2];
        
        verts=model.getVerts();
        faces=model.getFaces();
        scale=model.getScale();
        
        vertsNormal = self.getvertNormals(verts, faces);
        
        textureMap = model.textureMap;
        faceTextures = model.getFaceTextures();
        texture = model.getTexture();
        vertsTextcords = nan;
        
        light = [0,0,1];
        
        
        if(mode=="net"):
            #verts:
            for dot in verts:
                self.drawDot(scale * dot[0]+1000, scale * dot[1]+500);# минус для транспонирования - переворачивания!!!
                
            #faces:
            for face in faces:
                for i in range(len(face)):
                    v1_ind=face[i]-1;
                    v2_ind=face[(i+1)%len(face)]-1;# чтобы 2 вершина соединилась с 0 - перекрут
                
                    x1=verts[v1_ind][0] * scale + 1000;
                    y1=verts[v1_ind][1] * scale + 500;
                    x2=verts[v2_ind][0] * scale + 1000;
                    y2=verts[v2_ind][1] * scale + 500;
                    
                    self.drawLine(x1, y1, x2, y2,5,(255,0,0));
            
        elif(mode=="pol"):
            for face in faces:
                # добавить обработку на полигоны больше 3 вершин
                v1_ind=face[0]-1;
                v2_ind=face[1]-1;
                v3_ind=face[2]-1;
                    
                v1x=verts[v1_ind][0];
                v1y=verts[v1_ind][1];
                v1z=verts[v1_ind][2];
                
                v2x=verts[v2_ind][0];
                v2y=verts[v2_ind][1];
                v2z=verts[v2_ind][2];
        
                v3x=verts[v3_ind][0];
                v3y=verts[v3_ind][1];
                v3z=verts[v3_ind][2];
                
                
                n1 = vertsNormal[v1_ind];
                n2 = vertsNormal[v2_ind];
                n3 = vertsNormal[v3_ind];
                
                I = self.lightVerts(light, n1, n2, n3);
                
                #texturing:
                if(isTexture):
                    v1_texture = faceTextures[textureMap[face[0],face[1],face[2]][0]-1];
                    v2_texture = faceTextures[textureMap[face[0],face[1],face[2]][1]-1];
                    v3_texture = faceTextures[textureMap[face[0],face[1],face[2]][2]-1];
                
                    vertsTextcords = (v1_texture, v2_texture, v3_texture);
                
                Acos=self.cosNormal(light,v1x,v1y,v1z, v2x,v2y,v2z, v3x,v3y,v3z);
                if(Acos<0):
                    if(isSmooth):
                        self.drawTriangleSmooth(v1x,v1y,v1z, v2x,v2y,v2z, v3x,v3y,v3z,screenShift, vertsTextcords, texture, isTexture, I,scale,[-193,-191,-194]);
                    else:
                        self.drawTriangle(v1x,v1y,v1z,v2x,v2y,v2z,v3x,v3y,v3z,screenShift,scale,(-255*Acos,-255*Acos,-255*Acos));

    def drawTriangle(self,v1x, v1y, v1z, v2x, v2y, v2z, v3x, v3y, v3z, screenShift,scale, color=(255,255,255)):
        #ограничивающий треугольник:
        x0=(v1x/(v1z))*scale+screenShift[0];
        y0=(v1y/(v1z))*scale+screenShift[1];
        
        x1=(v2x/(v2z))*scale+screenShift[0];
        y1=(v2y/(v2z))*scale+screenShift[1];
        
        x2=(v3x/(v3z))*scale+screenShift[0];
        y2=(v3y/(v3z))*scale+screenShift[1]; 
        
        xmin=max(0,min(self.w, int(x0), int(x1), int(x2)));
        xmax=min(self.w,max(0, int(x0), int(x1), int(x2))+1);
        
        ymin=max(0,min(self.h, int(y0), int(y1), int(y2)));
        ymax=min(self.h,max(0, int(y0), int(y1), int(y2))+1);
        
        for y in range(ymin, ymax):
            for x in range(xmin,xmax):
                coords=self.baricentric(x, y, x0, y0, x1, y1, x2, y2);
                if(coords[0]>=0 and coords[1]>=0 and coords[2]>=0):
                    if(self.isDrawPixel(x, y, coords, (v1z,v2z,v3z))):
                        self.drawDot(x,y,color);
    def drawTriangleSmooth(self,v1x, v1y, v1z, v2x, v2y, v2z, v3x, v3y, v3z, screenShift, vtcords, texture, isTexture, I,scale, color):
        #ограничивающий треугольник:
        x0=(v1x/(v1z))*scale+screenShift[0];
        y0=(v1y/(v1z))*scale+screenShift[1];
        
        x1=(v2x/(v2z))*scale+screenShift[0];
        y1=(v2y/(v2z))*scale+screenShift[1];
        
        x2=(v3x/(v3z))*scale+screenShift[0];
        y2=(v3y/(v3z))*scale+screenShift[1]; 
        
        xmin=max(0,min(self.w, int(x0), int(x1), int(x2)));
        xmax=min(self.w,max(0, int(x0), int(x1), int(x2))+1);
        
        ymin=max(0,min(self.h, int(y0), int(y1), int(y2)));
        ymax=min(self.h,max(0, int(y0), int(y1), int(y2))+1);
        
        
        newcolor=[0,0,0];
        W=len(texture[0]);
        H=len(texture);
        
        for y in range(ymin, ymax):
            for x in range(xmin,xmax):
                coords=self.baricentric(x, y, x0, y0, x1, y1, x2, y2);
                if(coords[0]>=0 and coords[1]>=0 and coords[2]>=0):
                    if(self.isDrawPixel(x, y, coords, (v1z,v2z,v3z))):
    
                        if (isTexture):
                            xt=int(W*(coords[0]*vtcords[0][0]+coords[1]*vtcords[1][0]+coords[2]*vtcords[2][0]));
                            yt=int(H*(coords[0]*vtcords[0][1]+coords[1]*vtcords[1][1]+coords[2]*vtcords[2][1]));
                            colorText = texture[H-yt,xt];
                            color[0] = -1*colorText[0];
                            color[1] = -1*colorText[1];
                            color[2] = -1*colorText[2];
                    
                        newcolor[0] = color[0] * (coords[0] * I[0] + coords[1] * I[1] + coords[2] * I[2]);
                        newcolor[1] = color[1] * (coords[0] * I[0] + coords[1] * I[1] + coords[2] * I[2]);
                        newcolor[2] = color[2] * (coords[0] * I[0] + coords[1] * I[1] + coords[2] * I[2]);
                        
                        self.drawDot(x,y,newcolor);
    def baricentric(self,x:int, y:int, x0, y0, x1, y1, x2, y2):
        denominator= (x0-x2) * (y1-y2) - (x1-x2) * (y0-y2);
        if(denominator==0):
            return -1,-1,-1;
        lambda0 = ((x-x2) * (y1-y2) - (x1-x2) * (y-y2)) / denominator;
        lambda1 = ((x0-x2) * (y-y2) - (x-x2) * (y0-y2)) / denominator;
        lambda2 = 1.0-lambda0-lambda1;
    
        return (lambda0, lambda1, lambda2);
    def getvertNormals(self, verts, faces):
        vertsNormal = [];
        for i in range(len(verts)):
            vertsNormal.append(np.array([0.0,0.0,0.0]));
        for face in faces:
            v1_ind=face[0]-1;
            v2_ind=face[1]-1;
            v3_ind=face[2]-1;
                
            v1x=verts[v1_ind][0];
            v1y=verts[v1_ind][1];
            v1z=verts[v1_ind][2];
            
            v2x=verts[v2_ind][0];
            v2y=verts[v2_ind][1];
            v2z=verts[v2_ind][2];
    
            v3x=verts[v3_ind][0];
            v3y=verts[v3_ind][1];
            v3z=verts[v3_ind][2];
        
            normal = self.normal(v1x, v1y, v1z, v2x, v2y, v2z, v3x, v3y, v3z);
            
            vertsNormal[v1_ind] += normal;
            vertsNormal[v2_ind] += normal;
            vertsNormal[v3_ind] += normal;
        
        for i in range(len(vertsNormal)):
            temp = vertsNormal[i];
            vertsNormal[i] = temp/(self.norma(temp));
        #print (vertsNormal)
        return vertsNormal;
    def normal(self,x0,y0,z0,x1,y1,z1,x2,y2,z2):
        v1 = [x1-x2, y1-y2, z1-z2];
        v2 = [x1-x0, y1-y0, z1-z0];
        n = np.cross(v1, v2);
        return n;
    def lightVerts(self,light, n0, n1, n2):
        l = light;
        l0 = (np.dot(n0, l))/(self.norma(n0)*self.norma(l));
        l1 = (np.dot(n1, l))/(self.norma(n1)*self.norma(l));
        l2 = (np.dot(n2, l))/(self.norma(n2)*self.norma(l));
        
        return (l0,l1,l2);
    def cosNormal(self,light,x0,y0,z0,x1,y1,z1,x2,y2,z2):
        n=self.normal(x0,y0,z0,x1,y1,z1,x2,y2,z2);
        l=light;
        return (np.dot(n,l))/(self.norma(n)*self.norma(l));
    def norma(self,vector):
        s=0;
        for el in vector:
            s+=el**2;
        return sqrt(s);
    def isDrawPixel(self,x, y, BarCoords, Zcoords):
        z=BarCoords[0]*Zcoords[0] + BarCoords[1]*Zcoords[1] + BarCoords[2]*Zcoords[2];
        if(z<=self.zBuffer[y][x]):
            self.zBuffer[y][x]=z;
            return True;
        else:
            return False;        
    def drawLine(self, x0, y0, x1, y1, mode=5, color=(255,255,255)):
        if(mode == 1):
            step=1.0 / sqrt((x0-x1)**2 + (y0-y1)**2);
            t = 0;
            while(t - 1 < 1e-16):
                x=round((1.0-t)*x0 + t*x1);
                y=round((1.0-t)*y0 + t*y1);
                
                self.drawDot(x,y,color);
                t+=step;
                
        elif(mode == 2):
            delta=x1-x0;
            
            for x in range(int(x0),int(x1)):
                t=(x-x0) / delta;
                y=round((1.0-t)*y0 +t*y1);
                self.drawDot(x,y,color);
                
        elif(mode == 3):
            xchange=False;
            
            if(abs(x0-x1)<abs(y0-y1)):
                x0,y0=y0,x0;
                x1,y1=y1,x1;
                xchange=True;
            
            if(x0>x1):
                x0,x1=x1,x0;
                y0,y1=y1,y0;
                
            delta=x1-x0;
            
            for x in range(int(x0),int(x1)):
                t=(x-x0) / delta;
                y=round((1.0-t)*y0 +t*y1);
                
                if(xchange):
                    self.drawDot(y,x,color);
                else:
                    self.drawDot(x,y,color);
        
        elif(mode == 4):
            xchange=False;
            
            if(abs(x0-x1)<abs(y0-y1)):
                x0,y0=y0,x0;
                x1,y1=y1,x1;
                xchange=True;
                
            if(x0>x1):
                x0,x1=x1,x0;
                y0,y1=y1,y0;
                
            delta=x1-x0;
            dy=abs(y1-y0)/delta;
            derror=0.0;
            y_update= 1 if(y1>y0) else -1;
            y=y0;
            for x in range(int(x0),int(x1)):
                if(xchange):
                    self.drawDot(y,x,color);
                else:
                    self.drawDot(x,y,color);
                derror+=dy;
                if(derror>0.5):
                    derror-=1.0;
                    y+=y_update;
                    
        elif(mode == 5):
            xchange=False;
            
            if(abs(x0-x1)<abs(y0-y1)):
                x0,y0=y0,x0;
                x1,y1=y1,x1;
                xchange=True;
            
            if(x0>x1):
                x0,x1=x1,x0;
                y0,y1=y1,y0;
                
            delta=int(x1-x0);
            dy=2 * abs(y1-y0);
            derror = 0;
            y_update= 1 if(y1>y0) else -1;
            y = int(y0);
            for x in range(int(x0),int(x1)):
                if(xchange):
                    self.drawDot(y,x,color);
                else:
                    self.drawDot(x,y,color);
                derror += dy;
                if(derror>delta):
                    derror -= 2 * delta;
                    y += y_update;
    def drawStar(self, x_center, y_center, mode=5, color=(255,255,255)):
        for i in range(0,13):
            a=2*pi*i/13;
            x=x_center+95*cos(a);
            y=y_center+95*sin(a);
            self.drawLine(x_center,y_center,int(x),int(y),mode,color);
    
    def drawDot(self, x, y, color=(255,255,255)):
        self.screen[int(y)][int(x)] = color;
    
    def show(self):
        image=Image.fromarray(self.screen, mode='RGB');
        image=ImageOps.flip(image);
        image.save(self.File);


scale = 10**4 * 0.2;

model=Model([0,0,0],[0,0,0],"frog.obj","frog_texture.jpg",scale);
model.readObj();
model.RotateBase((-pi/2,pi,0));
model.Shift((0,-0.15,10.2));

rabbit=Model([0,0,0],[0,0,0],"frog.obj","frog_texture.jpg",scale);
rabbit.readObj();
rabbit.RotateQuaternion((-pi/2,pi,pi));
rabbit.Shift((0,-0.35,10.2));


d1=Display("frog_2.png",4000,4000);


isSmooth = True;
isTexture = True;
d1.drawModel(model,isSmooth,isTexture,0,"pol");
d1.drawModel(rabbit,isSmooth,isTexture,0,"pol");


print('done!');

d1.show();

























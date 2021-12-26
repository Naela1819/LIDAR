import tkinter as tk
from tkinter import filedialog as fd
from tkinter import *
import numpy as np
from PIL import ImageTk
import matplotlib.pyplot as plt
from matplotlib.backends._backend_tk import NavigationToolbar2Tk
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

                #====Interface====
#====Fenetre d'affichage====
fen=Tk()
fen.title("Application sur les données MMS")
fen.geometry("700x500+100+50")
fen.resizable(False,False)
#==== image de fond====
bg = ImageTk.PhotoImage(file="images/bk.jpg")
bg_image = Label(fen, image=bg).place(x=0, y=0, relwidth=1, relheight=1)
#====titre====
module_name = tk.Label(fen, text="Télédétection LIDAR", font=("Impact",19),fg="#223E63",bg="white").place(x=250,y=40)
pannel_title = tk.Label(fen, text="Programme de Géoréférencement Directe", font=("Impact",19),fg="#223E63",bg="white").place(x=140,y=90)
#====noms====
frame_names = Frame(fen,bg="white")
frame_names.place(x=500,y=370,height=85,width=170)
title = Label(frame_names, text="Réalisé par:", font=("Impact",13),fg="#223E63",bg="white").place(x=20,y=10)
name1 = Label(frame_names, text="EL IMRANI Nouhaila", font=("Goudy old style",10),fg="#1A2026",bg="white").place(x=20,y=40)
name2 = Label(frame_names, text="IRJA Malak", font=("Goudy old style",10),fg="#1A2026",bg="white").place(x=20,y=60)


            # ====Charger les donnees GPS====

def callGPS():
    global dataGPS
    calling = fd.askopenfile()
    dataGPS = np.loadtxt(calling.name,skiprows=1)
    return dataGPS

affiche_btn = Button(fen,text="Les données GPS",bg="#12325B",fg="white",
                     bd=0,padx=40,cursor="hand2",font=("times new roman",15), command=callGPS)
affiche_btn.place(x=100,y=190,width=220)

            # ====Charger les donnees Scanner====
def callScanner():
    global dataScanner
    filez = fd.askopenfilename(multiple=True)
    var = fen.tk.splitlist(filez)
    var1 = np.loadtxt(var[0], skiprows=0)
    var2 = np.loadtxt(var[1], skiprows=0)
    var3 = np.loadtxt(var[2], skiprows=0)
    var4 = np.loadtxt(var[3], skiprows=0)
    var5 = np.loadtxt(var[4], skiprows=0)
    var6 = np.loadtxt(var[5], skiprows=0)
    var7 = np.loadtxt(var[6], skiprows=0)
    var8 = np.loadtxt(var[7], skiprows=0)
    var9 = np.loadtxt(var[8], skiprows=0)
    var10= np.loadtxt(var[9], skiprows=0)
    # grouper les donnees:
    dataScanner = [var1,var2,var3,var4,var5,var6,var7,var8,var9,var10]
    return dataScanner
affiche_btn = Button(fen,text="Les données scanner",bg="#12325B",
                     fg="white",bd=0,padx=40,cursor="hand2",font=("times new roman",15), command= callScanner)
affiche_btn.place(x=100,y=270,width=220)


#Les fonctions de rotations :

def rotX(thetaX):
    rotX = np.array([[1, 0, 0], [0, np.cos(thetaX*(np.pi/180)), -np.sin(thetaX*(np.pi/180))],
                     [0, np.sin(thetaX*(np.pi/180)), np.cos(thetaX*(np.pi/180))]])
    return rotX
def rotY(thetaY):
    rotY = np.array([[np.cos(thetaY*(np.pi/180)), 0, np.sin(thetaY*(np.pi/180))], [0, 1, 0],
                         [-np.sin(thetaY*(np.pi/180)), 0, np.cos(thetaY*(np.pi/180))]])
    return rotY
def rotZ(thetaZ):
        rotZ = np.array([[np.cos(thetaZ*(np.pi/180)), -np.sin(thetaZ*(np.pi/180)), 0], [np.sin(thetaZ*(np.pi/180)),
                                                                np.cos(thetaZ*(np.pi/180)), 0], [0, 0, 1]])
        return rotZ


def Georeferencement_point(data_GPS, Data_scanner):
    Nbr_profile = 0
    Nbr_points = 0
    N = 0
    for i in range(len(Data_scanner)):
        N = N + len(Data_scanner[i])
        pts = np.zeros((N, 3))
    for i in range(len(Data_scanner)):
        Data = Data_scanner[i][Data_scanner[i][:, 1].argsort()]
        bras_levier = np.array([0.14, 0.249, -0.076])
        rotation_sc = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]])
        id_profile = 0
        for j in range(len(Data)):
            if Data[j, 1] == id_profile:
                L = rotX(data_GPS[Nbr_profile + 1, 7]).dot(rotY(data_GPS[Nbr_profile + 1, 8]))
                rotation_GPS = L.dot(rotZ((data_GPS[Nbr_profile + 1, 9])))
                x = Data[j, 2:5].reshape(-1, 1)
                y = bras_levier.reshape(-1, 1)
                z = rotation_sc
                pts[Nbr_points + j, :] = data_GPS[Nbr_profile + 1, 1:4] + (
                    rotation_GPS.dot(y + z.dot(x))).transpose()
            else:
                id_profile = id_profile + 1
                Nbr_profile = Nbr_profile + 1
                L = rotX(data_GPS[Nbr_profile + 1, 7]).dot(rotY(data_GPS[Nbr_profile + 1, 8]))
                rotation_GPS = L.dot(rotZ((data_GPS[Nbr_profile + 1, 9])))
                x = Data[j, 2:5].reshape(-1, 1)
                y = bras_levier.reshape(-1, 1)
                z = rotation_sc
                pts[Nbr_points + j, :] = data_GPS[Nbr_profile + 1, 1:4] + (rotation_GPS.dot(y + z.dot(x))).transpose()
        Nbr_points = Nbr_points + len(Data)
    AfficherData(pts)
    return pts


# ====fonction affichage des donnees====
def AfficherData(points):
    x = points[:, 0]
    y = points[:, 1]
    z = points[:, 2]
    fig = plt.figure(figsize=(150,150))
    ax = plt.axes(projection='3d')
    ax.scatter3D(x, y, z, c=z)
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')
    canvas = FigureCanvasTkAgg(fig, master=fen)
    toolbar = NavigationToolbar2Tk(canvas, fen)
    toolbar.update()
    canvas.get_tk_widget().pack()
    toolbar.update()


def fct_call():
    PTS_Georef = Georeferencement_point(dataGPS, dataScanner)
    AfficherData(PTS_Georef)
# ====Button afficher les donnees====
rst_btn = Button(fen,text="Géoréférencement directe",bg="#12325B",fg="white",
                 bd=0,padx=40,cursor="hand2",font=("times new roman",15), command= fct_call)
rst_btn.place(x=350,y=230)

fen.mainloop()


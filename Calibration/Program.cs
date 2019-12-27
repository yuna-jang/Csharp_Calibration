using System;
using System.IO;

namespace Calibration
{
    public struct struct_IOP_header
    {
        public double Nw;
        public double Nh;
        public double pp_x;
        public double pp_y;
        public double f_mm;
        public double Fw;
        public double Fh;
        public double Scale0;
        public double Scale1;
        public double f0;
        public double f1;
        public double k0;
        public double k1;
        public double k2;
        public double k3;
        public double k4;
        public double k_inv0;
        public double k_inv1;
        public double k_inv2;
        public double k_inv3;
        public double k_inv4;
        public double rect_scale;
        public double filter_fov;

        public BinaryReader binaryReader;
    }

    public struct struct_EOP_header
    {
        public double Mvec0;
        public double Mvec1;
        public double Mvec2;
        public double T0;
        public double T1;
        public double T2;
        public double[,] Mat;

        public BinaryReader binaryReader;
    }

    public struct struct_rph
    {
        public double roll_INS;
        public double pitch_INS;
        public double heading_INS;
    }

    class Program
    {
        public static struct_IOP_header IOP_header;
        public static struct_EOP_header EOP_header;
        public static struct_rph rph;
        public static string mfolder_base = @"D:\stryx_JYA_hdMap_dev\smms_fisheye\mBody_cam_2.bin";

        public static string path_depthMap_gpoint;
        public static double[] sPoint = new double[3];
        public static double[] INS_xyz = new double[3];
        public static double[] INS_rph = new double[3];
        public static double[,] INS_mat = new double[3, 3];
        public static double[] mXYZ = new double[3];

        public xyz2pix()
        {
            Console.WriteLine("Starting Calibation ...");
        }

        public double[] do_xyz2pix(double X, double Y, double Z, double x_tm, double y_tm, double height, double roll, double pitch, double heading, int icam)
        {
            class_sample_calibration CB = new class_sample_calibration();
            (IOP_header, EOP_header) = CB.OpenBinaryReader(IOP_header, EOP_header, mfolder_base, icam);
            sPoint = CB.Setxyz(X, Y, Z);  // object의 위치
            (INS_xyz, INS_rph) = CB.SetFromINS(x_tm, y_tm, height, roll, pitch, heading); // platform body의 위치 

            class_INS_calculation rotm = new class_INS_calculation();
            INS_mat = rotm.rotM_novatel(INS_rph);
            mXYZ = rotm.INS_Rmat(sPoint, INS_xyz, INS_mat);

            double[,] sXYZcam = new double[3, 1];
            sXYZcam = rotm.pixel_obj2cam(mXYZ, EOP_header, 1);

            double[,] sXYimg = rotm.pixel_cam2img_equidist(sXYZcam, IOP_header, 1);
            double[] XY_Pix;
            if (sXYimg != null)
                XY_Pix = new double[] { sXYimg[0, 0], sXYimg[1, 0] };
            else
                XY_Pix = new double[] { 0, 0 };
            return XY_Pix;
        }
    }

    public class class_sample_calibration
    {
        public class_sample_calibration()
        {
            Console.WriteLine("Create xyz2rowcol class");
        }

        public (struct_IOP_header, struct_EOP_header) OpenBinaryReader(struct_IOP_header IOP_header, struct_EOP_header EOP_header, string mfolder_base, int icam)
        {
            IOP_header.binaryReader = new BinaryReader(File.Open(mfolder_base, FileMode.Open));
            uint N_cam = IOP_header.binaryReader.ReadByte();
            for (int i = 0; i < icam; i++)
            {
                IOP_header.Nw = IOP_header.binaryReader.ReadDouble();
                IOP_header.Nh = IOP_header.binaryReader.ReadDouble();
                IOP_header.pp_x = IOP_header.binaryReader.ReadDouble();
                IOP_header.pp_y = IOP_header.binaryReader.ReadDouble();
                IOP_header.f_mm = IOP_header.binaryReader.ReadDouble();
                IOP_header.Fw = IOP_header.binaryReader.ReadDouble();
                IOP_header.Fh = IOP_header.binaryReader.ReadDouble();
                IOP_header.Scale0 = IOP_header.binaryReader.ReadDouble();
                IOP_header.Scale1 = IOP_header.binaryReader.ReadDouble();
                IOP_header.f0 = IOP_header.binaryReader.ReadDouble();
                IOP_header.f1 = IOP_header.binaryReader.ReadDouble();
                IOP_header.k0 = IOP_header.binaryReader.ReadDouble();
                IOP_header.k1 = IOP_header.binaryReader.ReadDouble();
                IOP_header.k2 = IOP_header.binaryReader.ReadDouble();
                IOP_header.k3 = IOP_header.binaryReader.ReadDouble();
                IOP_header.k4 = IOP_header.binaryReader.ReadDouble();
                IOP_header.k_inv0 = IOP_header.binaryReader.ReadDouble();
                IOP_header.k_inv1 = IOP_header.binaryReader.ReadDouble();
                IOP_header.k_inv2 = IOP_header.binaryReader.ReadDouble();
                IOP_header.k_inv3 = IOP_header.binaryReader.ReadDouble();
                IOP_header.k_inv4 = IOP_header.binaryReader.ReadDouble();
                IOP_header.rect_scale = IOP_header.binaryReader.ReadDouble();
                IOP_header.filter_fov = 180;
                EOP_header.Mvec0 = IOP_header.binaryReader.ReadDouble();
                EOP_header.Mvec1 = IOP_header.binaryReader.ReadDouble();
                EOP_header.Mvec2 = IOP_header.binaryReader.ReadDouble();
                EOP_header.T0 = IOP_header.binaryReader.ReadDouble();
                EOP_header.T1 = IOP_header.binaryReader.ReadDouble();
                EOP_header.T2 = IOP_header.binaryReader.ReadDouble();

                EOP_header.Mat = new double[3, 3];
                EOP_header.Mat[0, 0] = IOP_header.binaryReader.ReadDouble();
                EOP_header.Mat[1, 0] = IOP_header.binaryReader.ReadDouble();
                EOP_header.Mat[2, 0] = IOP_header.binaryReader.ReadDouble();
                EOP_header.Mat[0, 1] = IOP_header.binaryReader.ReadDouble();
                EOP_header.Mat[1, 1] = IOP_header.binaryReader.ReadDouble();
                EOP_header.Mat[2, 1] = IOP_header.binaryReader.ReadDouble();
                EOP_header.Mat[0, 2] = IOP_header.binaryReader.ReadDouble();
                EOP_header.Mat[1, 2] = IOP_header.binaryReader.ReadDouble();
                EOP_header.Mat[2, 2] = IOP_header.binaryReader.ReadDouble();

            }
            IOP_header.binaryReader.Close();

            return (IOP_header, EOP_header);
        }

        public double[] Setxyz(double X, double Y, double Z)
        {
            double[] ObjectPos = new double[3];
            ObjectPos[0] = X;
            ObjectPos[1] = Y;
            ObjectPos[2] = Z;
            return ObjectPos;
        }

        public (double[], double[]) SetFromINS(double x_tm, double y_tm, double z_tm, double roll_ins, double pitch_ins, double heading_ins)  //From mark_pos DB
        {
            double[] INS_xyz = new double[3];
            double[] INS_rph = new double[3];

            INS_xyz[0] = x_tm;
            INS_xyz[1] = y_tm;
            INS_xyz[2] = z_tm;

            INS_rph[0] = roll_ins;
            INS_rph[1] = pitch_ins;
            INS_rph[2] = heading_ins;

            return (INS_xyz, INS_rph);
        }

    }

    public class class_INS_calculation
    {
        public class_INS_calculation()
        {
            Console.WriteLine("INS_mat 생성중 .............");
        }

        public double[,] rotM_novatel(double[] rph)
        {
            double cR = Math.Cos(rph[0]);
            double sR = Math.Sin(rph[0]);

            double cP = Math.Cos(rph[1]);
            double sP = Math.Sin(rph[1]);

            double cY = Math.Cos(rph[2]);
            double sY = Math.Sin(rph[2]);

            double[,] rotM = new double[3, 3];
            rotM[0, 0] = cY * cR - sY * sP * sR;
            rotM[0, 1] = -sY * cP;
            rotM[0, 2] = cY * sR + sY * sP * cR;

            rotM[1, 0] = sY * cR + cY * sP * sR;
            rotM[1, 1] = cY * cP;
            rotM[1, 2] = sY * sR - cY * sP * cR;

            rotM[2, 0] = -cP * sR;
            rotM[2, 1] = sP;
            rotM[2, 2] = cP * cR;

            return rotM;
        }

        public double[] INS_Rmat(double[] sPoint, double[] INS_xyz, double[,] INS_mat)
        {
            double diff_obj2ins;
            double[] mXYZ = new double[3];
            for (int i = 0; i < sPoint.Length; i++)
            {
                diff_obj2ins = (sPoint[0] - INS_xyz[0]) * INS_mat[i, 0] + (sPoint[1] - INS_xyz[1]) * INS_mat[i, 1] + (sPoint[2] - INS_xyz[2]) * INS_mat[i, 2];
                mXYZ[i] = diff_obj2ins;
            }

            return mXYZ;
        }


        public double[,] pixel_obj2cam(double[] sXYZobj, struct_EOP_header EOP_header, int flag_filter)
        {
            double[] sT = new double[] { EOP_header.T0, EOP_header.T1, EOP_header.T2 };
            double[,] sMmat = Mat3d(EOP_header.Mvec0, EOP_header.Mvec1, EOP_header.Mvec2);

            double[,] sXYZobj0 = new double[3, 1];
            for (int i = 0; i < 3; i++)
            {
                sXYZobj0[i, 0] = sXYZobj[i] - sT[i];
            }
            double[,] sXYZcam = new double[3, 1];
            for (int i = 0; i < 3; i++)
            {
                sXYZcam[i, 0] = sMmat[i, 0] * sXYZobj0[0, 0] + sMmat[i, 1] * sXYZobj0[1, 0] + sMmat[i, 2] * sXYZobj0[2, 0];
                Console.WriteLine(sXYZcam[i, 0]);
            }
            if (flag_filter == 1)
            {
                if (sXYZcam[2, 0] < 0)
                {
                    sXYZcam = null;
                    return sXYZcam;
                }
            }
            else if (flag_filter == 2)
            {
                if (sXYZcam[2, 0] < -1)
                {
                    sXYZcam = null;
                    return sXYZcam;
                }
            }
            sXYZcam[0, 0] = -(sXYZcam[0, 0] / sXYZcam[2, 0]);
            sXYZcam[1, 0] = -(sXYZcam[1, 0] / sXYZcam[2, 0]);
            sXYZcam[2, 0] = -(sXYZcam[2, 0] / sXYZcam[2, 0]);

            return sXYZcam;
        }

        public double[,] Mat3d(double w, double p, double k)
        {
            double[,] Mwpk = new double[3, 3];

            Mwpk[0, 0] = Math.Cos(k) * Math.Cos(p);
            Mwpk[0, 1] = Math.Cos(w) * Math.Sin(k) + Math.Sin(w) * Math.Sin(p) * Math.Cos(k);
            Mwpk[0, 2] = Math.Sin(w) * Math.Sin(k) - Math.Cos(w) * Math.Sin(p) * Math.Cos(k);
            Mwpk[1, 0] = -Math.Cos(p) * Math.Sin(k);
            Mwpk[1, 1] = Math.Cos(w) * Math.Cos(k) - Math.Sin(w) * Math.Sin(p) * Math.Sin(k);
            Mwpk[1, 2] = Math.Sin(w) * Math.Cos(k) + Math.Cos(w) * Math.Sin(p) * Math.Sin(k);
            Mwpk[2, 0] = Math.Sin(p);
            Mwpk[2, 1] = -Math.Sin(w) * Math.Cos(p);
            Mwpk[2, 2] = Math.Cos(w) * Math.Cos(p);

            return Mwpk;
        }

        public double[,] pixel_cam2img_equidist(double[,] sXYZcam, struct_IOP_header IOP_header, int flag_filter)
        {
            if (sXYZcam != null)
            {
                double sR = Math.Sqrt(Math.Pow(sXYZcam[0, 0], 2) + Math.Pow(sXYZcam[1, 0], 2));
                double theta = Math.Atan(sR);

                if (flag_filter == 1)
                {
                    int mflag_filter = 170;
                    if (theta > (Math.PI / 180) * (mflag_filter / 2))
                        theta = 0;
                }

                double scaling = theta / sR;

                // Symbolic
                double[,] sXYimg = new double[2, 1];
                sXYimg[0, 0] = sXYZcam[0, 0] * scaling;
                sXYimg[1, 0] = sXYZcam[1, 0] * scaling;
                // Lens Distortion
                sR = Math.Sqrt(Math.Pow(sXYimg[0, 0], 2) + Math.Pow(sXYimg[1, 0], 2));
                double tR2 = sR * sR;
                double AA = IOP_header.k0 * tR2 + IOP_header.k1 * Math.Pow(tR2, 2) + IOP_header.k2 * Math.Pow(tR2, 3);
                double tXY = sXYimg[0, 0] * sXYimg[1, 0];
                double[,] sXYlensDistortion = new double[2, 1];
                sXYlensDistortion[0, 0] = sXYimg[0, 0] * AA + IOP_header.k3 * (tR2 + 2 * Math.Pow(sXYimg[0, 0], 2)) + 2 * IOP_header.k4 * tXY;
                sXYlensDistortion[1, 0] = sXYimg[1, 0] * AA + IOP_header.k4 * (tR2 + 2 * Math.Pow(sXYimg[1, 0], 2)) + 2 * IOP_header.k3 * tXY;

                sXYimg[0, 0] = sXYimg[0, 0] + sXYlensDistortion[0, 0];
                sXYimg[1, 0] = sXYimg[1, 0] + sXYlensDistortion[1, 0];
                // Scale : convert mm to pixel 
                sXYimg[0, 0] = sXYimg[0, 0] * (IOP_header.f_mm * IOP_header.Nw / IOP_header.Fw);
                sXYimg[1, 0] = sXYimg[1, 0] * (IOP_header.f_mm * IOP_header.Nw / IOP_header.Fw);

                // Principal point
                sXYimg[0, 0] = sXYimg[0, 0] + IOP_header.pp_x;
                sXYimg[1, 0] = sXYimg[1, 0] + IOP_header.pp_y;

                if (flag_filter == 1)
                {
                    if (sXYimg[0, 0] < 2 | sXYimg[1, 0] < 2 | sXYimg[0, 0] > IOP_header.Nw - 2 | sXYimg[1, 0] > IOP_header.Nw - 2)
                        sXYimg = null;
                }

                return sXYimg;
            }
            else
                return null;
        }
    }
}

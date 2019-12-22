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
        public static double[] sPoint = new double[3];
        public static double[] INS_xyz = new double[3];
        public static double[] INS_rph = new double[3];
        public static double[,] INS_mat = new double[3, 3];
        public static double[] mXYZ = new double[3];


        static void Main(string[] args)
        {
            class_sample_calibration CB = new class_sample_calibration(@"D:\stryx_JYA_hdMap_dev\smms_fisheye\00010\depth_00010_0_gPoint.bin");
            CB.OpenBinaryReader();
            sPoint = CB.Setxyz(189564.1200073242, 546414.7099945068, 42.070000228881840);  // object의 위치
            (INS_xyz, INS_rph) = CB.SetFromINS(189540.148202, 546441.107965, 33.144314, 0.003972, 0.015181, -3.116845); // platform body의 위치 

            class_INS_calculation rotm = new class_INS_calculation();
            INS_mat = rotm.rotM_novatel(INS_rph);
            mXYZ = rotm.INS_Rmat(sPoint, INS_xyz, INS_mat);
            Console.WriteLine($"INS_mat :\n{INS_mat[0, 0]} {INS_mat[0, 1]} {INS_mat[0, 2]} \n");
            Console.WriteLine($"{INS_mat[1, 0]} {INS_mat[1, 1]} {INS_mat[1, 2]} \n");
            Console.WriteLine($"{INS_mat[2, 0]} {INS_mat[2, 1]} {INS_mat[2, 2]} \n");
            Console.WriteLine($"sPoint : {sPoint[0]} {sPoint[1]} {sPoint[2]}");
            Console.WriteLine($"INS_xyz : {INS_xyz[0]} {INS_xyz[1]} {INS_xyz[2]}");
            Console.WriteLine($"mxyz : {mXYZ[0]} {mXYZ[1]} {mXYZ[2]}");

            Console.ReadLine();
        }
    }

    public class class_sample_calibration
    {
        public struct_IOP_header IOP_header;
        public struct_EOP_header EOP_header;
        public struct_rph rph;
        string mfolder_base = @"D:\stryx_JYA_hdMap_dev\smms_fisheye\mBody_cam_2.bin";
        public string path_depthMap_gpoint;

        public class_sample_calibration(string filepath)
        {
            Console.WriteLine("Create xyz2rowcol class");
            path_depthMap_gpoint = filepath;
        }

        public void OpenBinaryReader()
        {
            IOP_header.binaryReader = new BinaryReader(File.Open(mfolder_base, FileMode.Open));
            uint N_cam = IOP_header.binaryReader.ReadByte();
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

            Console.WriteLine($"N_cam :{N_cam}");
            Console.WriteLine($"IOP_header.Nw :{IOP_header.Nw}");
            Console.WriteLine($"IOP_header.Nh :{IOP_header.Nh }");
            Console.WriteLine($"IOP_header.pp_x :{IOP_header.pp_x }");
            Console.WriteLine($"IOP_header.pp_y :{IOP_header.pp_y }");
            Console.WriteLine($"IOP_header.f_mm :{IOP_header.f_mm}");
            Console.WriteLine($"IOP_header.Fw :{IOP_header.Fw }");
            Console.WriteLine($"IOP_header.Fh :{IOP_header.Fh }");
            Console.WriteLine($"IOP_header.rect_scale :{IOP_header.rect_scale}");
            Console.WriteLine($"EOP_header.Mvec :{EOP_header.Mvec0}, {EOP_header.Mvec1},{EOP_header.Mvec2}");
            Console.WriteLine($"EOP_header.T :{EOP_header.T0}, {EOP_header.T1}, {EOP_header.T2}");
            Console.WriteLine($"\n{EOP_header.Mat[0, 0]} {EOP_header.Mat[0, 1]} {EOP_header.Mat[0, 2]} \n");
            Console.WriteLine($"{EOP_header.Mat[1, 0]} {EOP_header.Mat[1, 1]} {EOP_header.Mat[1, 2]} \n");
            Console.WriteLine($"{EOP_header.Mat[2, 0]} {EOP_header.Mat[2, 1]} {EOP_header.Mat[2, 2]} \n");

            IOP_header.binaryReader.Close();
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
            double[] INS_xyz= new double[3];
            double[] INS_rph = new double[3];

            INS_xyz[0] = x_tm;
            INS_xyz[1] = y_tm;
            INS_xyz[2] = z_tm;

            INS_rph[0] = roll_ins;
            INS_rph[1] = pitch_ins;
            INS_rph[2] = heading_ins;

            return (INS_xyz, INS_rph);
        }

        public double[] sTranslation()
        {
            double[] sT = new double[] {EOP_header.T0, EOP_header.T1, EOP_header.T2};
            double ;
        }

        public double[] Mat3d(double w, double p, double k)
        {
            double[,] Mw = { { 1, 0, 0 }, { 0, Math.Cos(w), Math.Sin(w)}, { 0, -Math.Sin(w), Math.Cos(w) } };
            double[,] Mp = { { Math.Cos(p), 0, -Math.Sin(p)}, { 0,1,0}, {Math.Sin(p), 0, Math.Cos(p) } };
            double[,] Mk = { { Math.Cos(k), Math.Sin(k), 0},{ -Math.Sin(k), Math.Cos(k), 0},{ 0, 0, 1 } };

            double[,] Mwpk = new double[,] { };

            for(int i = 0; i < 3; i++)
            {
                for (int j = 0; j < 3; j++)
                {
                    Mwpk[i, k] = Mk[i, k] * Mp[k, i];
                    
                }
            }
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
                diff_obj2ins = (sPoint[0] - INS_xyz[0]) * INS_mat[i,0] + (sPoint[1] - INS_xyz[1]) * INS_mat[i,1] + (sPoint[2] - INS_xyz[2]) * INS_mat[i,2];
                mXYZ[i] = diff_obj2ins;
            }

            return mXYZ;
        }
    }
}

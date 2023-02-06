using System;
using System.Collections;
using System.Collections.Generic;

using Rhino;
using Rhino.Geometry;

using Grasshopper;
using Grasshopper.Kernel;
using Grasshopper.Kernel.Data;
using Grasshopper.Kernel.Types;

// <Custom using>
using System.Drawing;
using System.Threading;
using System.Threading.Tasks;
using System.Collections.Concurrent;
using System.Linq;
using Rhino.Collections;
//using Noises;
using Rhino.Render;
using System.Diagnostics;
using System.Windows.Forms;
using System.Security.Cryptography;
// </Custom using>


/// <summary>
/// This class will be instantiated on demand by the Script component.
/// </summary>
public class Script_Instance : GH_ScriptInstance
{
    #region Utility functions
    /// <summary>Print a String to the [Out] Parameter of the Script component.</summary>
    /// <param name="text">String to print.</param>
    private void Print(string text) { __out.Add(text); }
    /// <summary>Print a formatted String to the [Out] Parameter of the Script component.</summary>
    /// <param name="format">String format.</param>
    /// <param name="args">Formatting parameters.</param>
    private void Print(string format, params object[] args) { __out.Add(string.Format(format, args)); }
    /// <summary>Print useful information about an object instance to the [Out] Parameter of the Script component. </summary>
    /// <param name="obj">Object instance to parse.</param>
    private void Reflect(object obj) { __out.Add(GH_ScriptComponentUtilities.ReflectType_CS(obj)); }
    /// <summary>Print the signatures of all the overloads of a specific method to the [Out] Parameter of the Script component. </summary>
    /// <param name="obj">Object instance to parse.</param>
    private void Reflect(object obj, string method_name) { __out.Add(GH_ScriptComponentUtilities.ReflectType_CS(obj, method_name)); }
    #endregion

    #region Members
    /// <summary>Gets the current Rhino document.</summary>
    private RhinoDoc RhinoDocument;
    /// <summary>Gets the Grasshopper document that owns this script.</summary>
    private GH_Document GrasshopperDocument;
    /// <summary>Gets the Grasshopper script component that owns this script.</summary>
    private IGH_Component Component;
    /// <summary>
    /// Gets the current iteration count. The first call to RunScript() is associated with Iteration==0.
    /// Any subsequent call within the same solution will increment the Iteration count.
    /// </summary>
    private int Iteration;
    #endregion

    /// <summary>
    /// This procedure contains the user code. Input parameters are provided as regular arguments, 
    /// Output parameters as ref arguments. You don't have to assign output parameters, 
    /// they will have a default value.
    /// </summary>
    private void RunScript(bool newSS, bool iReset, bool iGo, int method, bool showBri, bool showTrails, bool showPoints, bool showField, bool debug, int Res, int Length, int Width, int Height, List<System.Object> P, List<Point3d> iPoint, List<Point3d> AttrPoint, List<Curve> AttrCurve, Box obstacle, double foodRadius, int tL, int agentsAtIt, int lifeTime, double sI, double dR, double eR, double sensDist, double sensAng, ref object points, ref object vectors, ref object oBri, ref object sensors, ref object trails, ref object field, ref object A)
    {
        // <Custom code>

        List<Point3d> pList = P.Select(p => (Point3d)p).ToList();

        switch (method)
        {
            case 0:

                if (newSS) SS = new SpatialSubdivision(Res, Length, Width, Height, dR, eR, foodRadius, AttrPoint, AttrCurve, obstacle);

                if (iPoint == null) return;

                if (AS == null || SS.points.Count != SS.scalarField.Length)
                {
                    SS = new SpatialSubdivision(Res, Length, Width, Height, dR, eR, foodRadius, AttrPoint, AttrCurve, obstacle);
                    AS = new AgentSystem(iPoint, SS);
                }

                if (iReset)
                {
                    SS.RestoreScalarField();
                    AS = new AgentSystem(iPoint, SS);
                    counter = 0;
                }

                if (iGo)
                {
                    Random rnd = new Random();

                    for (int i = 0; i < iPoint.Count; i++)
                        for (int j = 0; j < agentsAtIt; j++) AS.Agents.Add(new Agent(AS, iPoint[i], RandomVectors(iPoint, rnd.Next(0, 100))[i]));


                    foreach (Agent agent in AS.Agents) agent.life++;


                    for (int i = 0; i < AS.Agents.Count; i++)
                        if (AS.Agents[i].life > lifeTime) AS.Agents.RemoveAt(i);


                    AS.seekIntensity = sI;
                    AS.sensDist = sensDist;
                    AS.sensAng = sensAng;
                    SS.diffusionRate = (float)dR;
                    SS.evaporationRate = (float)eR;


                    AS.UpdateJones();
                    SS.Update();
                    counter++;

                    Component.ExpireSolution(true);
                }
                break;

            /********************************************************************************************/

            case 1:

                if (newSS) SS = new SpatialSubdivision(Res, Length, Width, Height, dR, eR, foodRadius, AttrPoint, AttrCurve, obstacle);

                if (P == null) return;

                if (AS == null || SS.points.Count != SS.scalarField.Length)
                {
                    SS = new SpatialSubdivision(Res, Length, Width, Height, dR, eR, foodRadius, AttrPoint, AttrCurve, obstacle);
                    AS = new AgentSystem(pList, SS);
                }

                if (iReset)
                {
                    SS.RestoreScalarField();
                    AS = new AgentSystem(pList, SS);
                    counter = 0;
                }

                if (iGo)
                {
                    AS.seekIntensity = sI;
                    AS.sensDist = sensDist;
                    AS.sensAng = sensAng;
                    SS.diffusionRate = (float)dR;
                    SS.evaporationRate = (float)eR;


                    AS.UpdateJones();
                    SS.Update();
                    counter++;



                    Component.ExpireSolution(true);
                }
                break;
        }

        /********************************************************************************************/

        //trails
        trs = new Polyline[AS.Agents.Count];

        Parallel.For(0, AS.Agents.Count, i =>
          {
              if (AS.Agents[i].trail.IsValid) trs[i] = AS.Agents[i].trail;
              else trs[i] = null;

              if (AS.Agents[i].trail.Length > tL)
                  AS.Agents[i].trail.RemoveAt(0);
          });

        if (showTrails) trails = trs;

        // particles positions and velocites
        if (showPoints)
        {
            AS.GetPointsVectors(out pts, out vecs);
            points = pts;
            vectors = vecs;
        }

        if (showBri) oBri = SS.GetColoredPoints();
        if (showField) field = SS.GetScalarField();
        if (debug) sensors = AS.SensorsOut();

        Print(counter.ToString());
        // </Custom code>
    }

    // <Custom additional code> 

    public AgentSystem AS;
    public SpatialSubdivision SS;
    public GH_Point[] pts;
    public GH_Vector[] vecs;
    public Polyline[] trs;
    public int counter;

    /********************************************************************************************/
    public class AgentSystem
    {
        public List<Agent> Agents;
        public double seekRadius;
        public double seekIntensity;
        public double sensDist;
        public double sensAng;
        public double gravityForce;
        public double BoundingBoxSize;
        public double ContainmentStrength;
        public double CurlNoiseStrength;
        public SpatialSubdivision SS;

        public AgentSystem(List<Point3d> positions, SpatialSubdivision SS)
        {
            this.SS = SS;
            Agents = new List<Agent>();

            for (int i = 0; i < positions.Count; i++) Agents.Add(new Agent(this, positions[i], RandomVector(positions, i)));

            BoundingBoxSize = SS.GetBoxSize();
            ContainmentStrength = 2.0;
        }

        //__________________________________________________//

        public void UpdateJones()
        {

            Parallel.ForEach(Agents, agent =>
             {
                 agent.currentCPindex = SS.LookUp2(agent.O);
                 agent.SeekAngVis();
             });

            foreach (Agent agent in Agents)
                SS.scalarField[agent.currentCPindex] = (float)Math.Min(0.8, SS.scalarField[agent.currentCPindex] + agent.PheroStrength);

            Parallel.ForEach(Agents, agent =>
            {
                agent.Update();
            });
        }


        public void GetPointsVectors(out GH_Point[] pts, out GH_Vector[] vecs)
        {
            pts = new GH_Point[Agents.Count];
            vecs = new GH_Vector[Agents.Count];

            for (int i = 0; i < Agents.Count; i++)
            {
                pts[i] = new GH_Point(Agents[i].O);
                vecs[i] = new GH_Vector(Agents[i].vel);
            }
        }


        public DataTree<GH_Vector> SensorsOut()
        {
            DataTree<GH_Vector> sensOut = new DataTree<GH_Vector>();

            for (int i = 0; i < Agents.Count; i++)
            {
                GH_Path path = new GH_Path(i);
                sensOut.Add(new GH_Vector(Agents[i].sensorC), path);

                sensOut.Add(new GH_Vector(Agents[i].sensorL), path);
                sensOut.Add(new GH_Vector(Agents[i].sensorR), path);
                sensOut.Add(new GH_Vector(Agents[i].sensorD), path);
                sensOut.Add(new GH_Vector(Agents[i].sensorU), path);

                sensOut.Add(new GH_Vector(Agents[i].sensorLC), path);
                sensOut.Add(new GH_Vector(Agents[i].sensorRC), path);
                sensOut.Add(new GH_Vector(Agents[i].sensorUC), path);
                sensOut.Add(new GH_Vector(Agents[i].sensorDC), path);

                sensOut.Add(new GH_Vector(Agents[i].sensorDR), path);
                sensOut.Add(new GH_Vector(Agents[i].sensorDCRC), path);
                sensOut.Add(new GH_Vector(Agents[i].sensorRU), path);
                sensOut.Add(new GH_Vector(Agents[i].sensorRCUC), path);
                sensOut.Add(new GH_Vector(Agents[i].sensorUL), path);
                sensOut.Add(new GH_Vector(Agents[i].sensorUCLC), path);
                sensOut.Add(new GH_Vector(Agents[i].sensorLD), path);
                sensOut.Add(new GH_Vector(Agents[i].sensorLCDC), path);
            }
            return sensOut;
        }
    }

    /********************************************************************************************/

    public class Agent
    {
        public Point3d O;
        public Vector3d X;
        public Vector3d Y;
        public Vector3d Z;
        public Vector3d vel;
        public Vector3d desiredVel;
        public Vector3d acc;
        public Vector3d sensorC, sensorL, sensorR, sensorU, sensorD, sensorLC, sensorRC, sensorUC, sensorDC, sensorDR, sensorDCRC, sensorRU, sensorRCUC, sensorUL, sensorUCLC, sensorLD, sensorLCDC;
        public Polyline trail;
        public double visAng;
        public double MaxSpeed;
        public float PheroStrength;
        public int currentCPindex;
        public int life;
        public AgentSystem aSystem;

        public Agent(AgentSystem aSystem, Point3d O, Vector3d vel)
        {
            this.aSystem = aSystem;
            this.O = O;
            this.vel = vel;
            X.Unitize();
            X = vel;
            Vector3d oV = new Vector3d(this.O);
            oV.Unitize();
            Y = Vector3d.CrossProduct(oV, this.X);
            Z = Vector3d.CrossProduct(X, Y);
            trail = new Polyline();
            MaxSpeed = 1.5;
            PheroStrength = 0.1f;
            visAng = aSystem.sensAng;
        }

        //__________________________________________________//

        public void Update()
        {
            //Containment();
            vel = 0.7 * vel + 0.3 * acc;

            if (vel.Length > MaxSpeed)
            {
                vel.Unitize();
                vel *= MaxSpeed;
            }
            if (vel.Length < 1)
            {
                vel.Unitize();
            }

            O += vel;

            trail.Add(O);
        }


        public void SeekAngVis()
        {
            acc = Vector3d.Zero;

            Plane pln = BuildPlaneSensors();
            visAng = aSystem.sensAng;
            Vector3d rotAxis = pln.ZAxis;
            Vector3d rotAxis2 = pln.YAxis;

            sensorC = VectorAmp(vel, aSystem.sensDist);

            sensorL = new Vector3d(sensorC);
            sensorLC = new Vector3d(sensorC);
            sensorR = new Vector3d(sensorC);
            sensorRC = new Vector3d(sensorC);
            sensorU = new Vector3d(sensorC);
            sensorUC = new Vector3d(sensorC);
            sensorD = new Vector3d(sensorC);
            sensorDC = new Vector3d(sensorC);

            sensorDR = new Vector3d(sensorC);
            sensorDCRC = new Vector3d(sensorC);
            sensorRU = new Vector3d(sensorC);
            sensorRCUC = new Vector3d(sensorC);
            sensorUL = new Vector3d(sensorC);
            sensorUCLC = new Vector3d(sensorC);
            sensorLD = new Vector3d(sensorC);
            sensorLCDC = new Vector3d(sensorC);


            sensorL.Rotate(visAng, rotAxis);
            sensorLC.Rotate(visAng * 0.5, rotAxis);
            sensorR.Rotate(-visAng, rotAxis);
            sensorRC.Rotate(-visAng * 0.5, rotAxis);
            sensorD.Rotate(visAng, rotAxis2);
            sensorDC.Rotate(visAng * 0.5, rotAxis2);
            sensorU.Rotate(-visAng, rotAxis2);
            sensorUC.Rotate(-visAng * 0.5, rotAxis2);

            sensorDR.Rotate(visAng, rotAxis2);
            sensorDR.Rotate(Math.PI * 0.25, pln.XAxis);

            sensorDCRC.Rotate(visAng * 0.5, rotAxis2);
            sensorDCRC.Rotate(Math.PI * 0.25, pln.XAxis);

            sensorRU.Rotate(-visAng, rotAxis);
            sensorRU.Rotate(Math.PI * 0.25, pln.XAxis);

            sensorRCUC.Rotate(-visAng * 0.5, rotAxis);
            sensorRCUC.Rotate(Math.PI * 0.25, pln.XAxis);

            sensorUL.Rotate(-visAng, rotAxis2);
            sensorUL.Rotate(Math.PI * 0.25, pln.XAxis);

            sensorUCLC.Rotate(-visAng * 0.5, rotAxis2);
            sensorUCLC.Rotate(Math.PI * 0.25, pln.XAxis);

            sensorLD.Rotate(visAng, rotAxis);
            sensorLD.Rotate(Math.PI * 0.25, pln.XAxis);

            sensorLCDC.Rotate(visAng * 0.5, rotAxis);
            sensorLCDC.Rotate(Math.PI * 0.25, pln.XAxis);


            int pL, pR, pC, pD, pU, pLC, pRC, pUC, pDC, pDR, pDCRC, pRU, pRCUC, pUL, pUCLC, pLD, pLCDC;
            double briL, briR, briC, briD, briU, briLC, briRC, briUC, briDC, briDR, briDCRC, briRU, briRCUC, briUL, briUCLC, briLD, briLCDC;


            pC = aSystem.SS.LookUp2(O + sensorC);

            pL = aSystem.SS.LookUp2(O + sensorL);
            pR = aSystem.SS.LookUp2(O + sensorR);
            pD = aSystem.SS.LookUp2(O + sensorD);
            pU = aSystem.SS.LookUp2(O + sensorU);

            pLC = aSystem.SS.LookUp2(O + sensorLC);
            pRC = aSystem.SS.LookUp2(O + sensorRC);
            pUC = aSystem.SS.LookUp2(O + sensorUC);
            pDC = aSystem.SS.LookUp2(O + sensorDC);

            pDR = aSystem.SS.LookUp2(O + sensorDR);
            pDCRC = aSystem.SS.LookUp2(O + sensorDCRC);
            pRU = aSystem.SS.LookUp2(O + sensorRU);
            pRCUC = aSystem.SS.LookUp2(O + sensorRCUC);
            pUL = aSystem.SS.LookUp2(O + sensorUL);
            pUCLC = aSystem.SS.LookUp2(O + sensorUCLC);
            pLD = aSystem.SS.LookUp2(O + sensorLD);
            pLCDC = aSystem.SS.LookUp2(O + sensorLCDC);


            briC = aSystem.SS.scalarField[pC];

            briL = aSystem.SS.scalarField[pL];
            briR = aSystem.SS.scalarField[pR];
            briD = aSystem.SS.scalarField[pD];
            briU = aSystem.SS.scalarField[pU];

            briLC = aSystem.SS.scalarField[pLC];
            briRC = aSystem.SS.scalarField[pRC];
            briUC = aSystem.SS.scalarField[pUC];
            briDC = aSystem.SS.scalarField[pDC];

            briDR = aSystem.SS.scalarField[pDR];
            briDCRC = aSystem.SS.scalarField[pDCRC];
            briRU = aSystem.SS.scalarField[pRU];
            briRCUC = aSystem.SS.scalarField[pRCUC];
            briUL = aSystem.SS.scalarField[pUL];
            briUCLC = aSystem.SS.scalarField[pUCLC];
            briLD = aSystem.SS.scalarField[pLD];
            briLCDC = aSystem.SS.scalarField[pLCDC];

            // find brightnest sensor
            Vector3d direction = new Vector3d();
            double[] briAll = new double[] { briC, briL, briR, briD, briU, briLC, briRC, briUC, briDC, briDR, briDCRC, briRU, briRCUC, briUL, briUCLC, briLD, briLCDC };
            Vector3d[] sensorAll = new Vector3d[] { sensorC, sensorL, sensorR, sensorD, sensorU, sensorLC, sensorRC, sensorUC, sensorDC, sensorDR, sensorDCRC, sensorRU, sensorRCUC, sensorUL, sensorUCLC, sensorLD, sensorLCDC };

            for (int i = 0; i < briAll.Length; i++) if (briAll[i] == briAll.Max()) direction = sensorAll[i];

            desiredVel += aSystem.SS.points[aSystem.SS.LookUp2(O + direction)] - O;

            desiredVel.Unitize();
            desiredVel *= MaxSpeed;
            acc = (desiredVel - vel) * aSystem.seekIntensity;
        }

        public Plane BuildPlaneSensors()
        {
            return new Plane(O, vel, Y);
        }

        public void Containment()
        {
            /*if (O.X < 0 || O.X > aSystem.BoundingBoxSize || O.Y < 0 || O.Y > aSystem.BoundingBoxSize || O.Z < 0 || O.Z > aSystem.BoundingBoxSize)
                desiredVel = aSystem.SS.B.Center - O;

            desiredVel.Unitize();
            desiredVel *= aSystem.ContainmentStrength;*/


            double distance = O.DistanceTo(aSystem.SS.bound.Center);
            if (distance > aSystem.SS.bound.Radius)
            {
                desiredVel = (aSystem.SS.bound.Center - O) * (distance / aSystem.SS.bound.Radius);
                desiredVel.Unitize();
                desiredVel *= aSystem.ContainmentStrength;
            }
        }

    }

    public class SpatialSubdivision
    {
        public Box B;
        public Sphere bound;
        public Box obstacles;
        public Box[,,] cells;
        public List<Point3d> points;
        public float[] scalarField;
        public int resolution;
        public int dim1;
        public int dim2;
        public int dim3;
        public int length;
        public int width;
        public int height;
        public float diffusionRate;
        public float evaporationRate;
        readonly float[] initVal;
        readonly int[][] NeighbourMap;
        readonly float[] DiffusionWeights;
        public List<Point3d> chemPoint;
        public List<Curve> chemCurve;
        public double foodRadius;
        public int[,,] env;
        RTree RTree;


        public SpatialSubdivision(int resolution, int length, int width, int height, double diffusionRate, double evaporationRate, double foodRadius, List<Point3d> chemPoint, List<Curve> chemCurve, Box obstacles)
        {
            this.resolution = resolution;
            this.length = length;
            this.width = width;
            this.height = height;
            dim1 = length / resolution;
            dim2 = width / resolution;
            dim3 = height / resolution;
            this.diffusionRate = (float)diffusionRate;
            this.evaporationRate = (float)evaporationRate;
            this.chemPoint = chemPoint;
            this.obstacles = obstacles;
            this.chemCurve = chemCurve;
            cells = InitCells();
            points = InitPoints();
            RTree = RTree.CreateFromPointArray(points);
            scalarField = PopulateScalarField();
            initVal = PopulateScalarField();
            NeighbourMap = BuildNeighboursMapRTree();
            DiffusionWeights = CalculateDiffusionWeights();
            B = new Box(Plane.WorldXY, new Interval(0, this.length), new Interval(0, this.width), new Interval(0, this.height));
            bound = new Sphere(B.Center, GetBoxSize() * 0.5);
            this.foodRadius = foodRadius;
            env = InitEnv();
        }

        public void Update()
        {
            Diffusion();
            EvaporateField();
            InsertChemPoint();
            InsertChemCurve();
            //InsertObstacle();
        }

        public double GetBoxSize()
        {
            Point3d[] corners = B.GetCorners();
            double size = corners[0].DistanceTo(corners[1]);
            return size;
        }

        public int[,,] InitEnv()
        {
            env = new int[dim1, dim2, dim3];
            Parallel.For(0, dim1, i =>
            {
                for (int j = 0; j < dim2; j++)
                    for (int k = 0; k < dim3; k++)
                        env[i, j, k] = points.IndexOf(cells[i,j,k].Center);
            });
            return env;
        }

        public Box[,,] InitCells()
        {
            cells = new Box[dim1, dim2, dim3];
            Interval interval = new Interval(0, resolution);
            Parallel.For(0, dim1, i =>
            {
                for (int j = 0; j < dim2; j++)
                    for (int k = 0; k < dim3; k++)
                        cells[i, j, k] = new Box(new Plane(new Point3d(i, j, k), Vector3d.ZAxis), interval, interval, interval);
            });
            return cells;
        }

        public List<Point3d> InitPoints()
        {

            points = new List<Point3d>();

            for (int i = 0; i < dim1; i++)
                for (int j = 0; j < dim2; j++)
                    for (int k = 0; k < dim3; k++)
                        points.Add(cells[i, j, k].Center);

            return points;
        }


        public int[] GetCPointsRTree(int q)
        {
            List<int> closestInd = new List<int>();
            double distance = points[0].DistanceTo(points[1]) * 1.75;

            RTree.Search(new Sphere(points[q], distance), (sender, e) =>
            {
                if (e.Id != q) closestInd.Add(e.Id);
            });

            return closestInd.ToArray();
        }

        public float[] PopulateScalarField()
        {
            float[] scalarField = new float[points.Count];

            Parallel.For(0, points.Count, i =>
             {
                 scalarField[i] = 0.0f;
             });

            return scalarField;
        }

        public void InsertChemPoint()
        {

            int foodInd = new int();

            Parallel.ForEach(chemPoint, cP =>
            {
                Sphere sph = new Sphere(cP, foodRadius);
                for (int i = 0; i < points.Count; i++)
                {
                    if (points[i].DistanceTo(sph.Center) < sph.Radius)
                    {
                        foodInd = LookUp2(points[i]);
                        scalarField[foodInd] = 1;
                    }
                }
            });

        }

        public void InsertChemCurve()
        {
            int foodInd = new int();
            Parallel.ForEach(chemCurve, crv =>
             {
                 double[] curveParameters = crv.DivideByCount(20, true);
                 Point3d[] curvePoints = new Point3d[curveParameters.Length];
                 for (int i = 0; i < curveParameters.Length; i++)
                 {
                     curvePoints[i] = crv.PointAt(curveParameters[i]);

                     foodInd = LookUp2(curvePoints[i]);
                     scalarField[foodInd] = 1;
                 }
             });

        }

        public void InsertObstacle()
        {

            int obsInd = new int();

            Parallel.For(0, points.Count, i =>
              {
                  if (obstacles.Contains(points[i]))
                  {
                      obsInd = LookUp2(points[i]);
                      scalarField[obsInd] = 0;
                  }
              });

        }

        public void EvaporateField()
        {
            Parallel.For(0, scalarField.Length, i =>
            {
                scalarField[i] *= evaporationRate;
            });
        }

        public void RestoreScalarField()
        {
            Parallel.For(0, scalarField.Length, i =>
            {
                scalarField[i] = initVal[i];
            });
        }


        public int[][] BuildNeighboursMapRTree()
        {
            int[][] NeighbourMap = new int[points.Count][];


            for (int i = 0; i < points.Count; i++)
                NeighbourMap[i] = GetCPointsRTree(i);

            return NeighbourMap;

        }

        public float[] CalculateDiffusionWeights()
        {
            float[] Weights = new float[NeighbourMap.Length];

            Parallel.For(0, NeighbourMap.Length, i =>
            {
                Weights[i] = 1 / (float)NeighbourMap[i].Length;
            });

            return Weights;
        }

        public float[] Diffusion()
        {
            float[] newVal = new float[scalarField.Length];

            Parallel.For(0, scalarField.Length, i =>
            {

                float neighVal = 0;

                for (int j = 0; j < NeighbourMap[i].Length; j++)
                    neighVal += scalarField[NeighbourMap[i][j]] * DiffusionWeights[i];

                newVal[i] = scalarField[i] * (1 - diffusionRate) + neighVal * diffusionRate;

            });

            Parallel.For(0, scalarField.Length, i =>
            {
                scalarField[i] = newVal[i];
            });

            return scalarField;
        }


        public Point3d LookUp(Point3d point)
        {
            int d1 = LimitToRange((int)(point.X / resolution), 0, dim1 - 1);
            int d2 = LimitToRange((int)(point.Y / resolution), 0, dim2 - 1);
            int d3 = LimitToRange((int)(point.Z / resolution), 0, dim3 - 1);

            return cells[d1, d2, d3].Center;
        }

        public int LookUp2(Point3d point)
        {
            int d1 = LimitToRange((int)(point.X / resolution), 0, dim1 - 1);
            int d2 = LimitToRange((int)(point.Y / resolution), 0, dim2 - 1);
            int d3 = LimitToRange((int)(point.Z / resolution), 0, dim3 - 1);

            return env[d1, d2, d3];
        }

        public GH_Colour[] GetColoredPoints()
        {
            GH_Colour[] outColors = new GH_Colour[scalarField.Length];

            Parallel.For(0, scalarField.Length, i =>
            {
                int c = (int)(scalarField[i] * 255);
                outColors[i] = new GH_Colour(Color.FromArgb(c, c, c));
            });

            return outColors;
        }

        public GH_Number[] GetScalarField()
        {
            GH_Number[] outField = new GH_Number[scalarField.Length];

            Parallel.For(0, scalarField.Length, i =>
            {
                outField[i] = new GH_Number(scalarField[i]);
            });

            return outField;
        }
    }


    #region Utilities

    public static Vector3d VectorAmp(Vector3d v, double Amplitude)
    {
        v.Unitize();
        return (v * Amplitude);
    }

    public static Vector3d RandomVector(List<Point3d> pts, int seed)
    {
        Vector3d vcs = new Vector3d();
        Random rnd = new Random(seed);
        for (int i = 0; i < pts.Count; i++)
        {
            Vector3d r = new Vector3d((rnd.NextDouble() * 2) - 1, (rnd.NextDouble() * 2) - 1, (rnd.NextDouble() * 2) - 1);
            vcs = r;
        }
        return vcs;
    }

    public static List<Vector3d> RandomVectors(List<Point3d> pts, int seed)
    {
        List<Vector3d> vcs = new List<Vector3d>();
        Random rnd = new Random(seed);
        for (int i = 0; i < pts.Count; i++)
        {
            Vector3d r = new Vector3d((rnd.NextDouble() * 2) - 1, (rnd.NextDouble() * 2) - 1, (rnd.NextDouble() * 2) - 1);
            vcs.Add(r);
        }
        return vcs;
    }

    public static int LimitToRange(int value, int inclusiveMinimum, int inlusiveMaximum)
    {
        if (value >= inclusiveMinimum)
        {
            if (value <= inlusiveMaximum)
            {
                return value;
            }
            else
                return inclusiveMinimum;
        }
        else
            return inlusiveMaximum;
    }


    #endregion

    // </Custom additional code> 

    private List<string> __err = new List<string>(); //Do not modify this list directly.
    private List<string> __out = new List<string>(); //Do not modify this list directly.
    private RhinoDoc doc = RhinoDoc.ActiveDoc;       //Legacy field.
    private IGH_ActiveObject owner;                  //Legacy field.
    private int runCount;                            //Legacy field.

    public override void InvokeRunScript(IGH_Component owner, object rhinoDocument, int iteration, List<object> inputs, IGH_DataAccess DA)
    {
        //Prepare for a new run...
        //1. Reset lists
        this.__out.Clear();
        this.__err.Clear();

        this.Component = owner;
        this.Iteration = iteration;
        this.GrasshopperDocument = owner.OnPingDocument();
        this.RhinoDocument = rhinoDocument as Rhino.RhinoDoc;

        this.owner = this.Component;
        this.runCount = this.Iteration;
        this.doc = this.RhinoDocument;

        //2. Assign input parameters
        bool newVE = default(bool);
        if (inputs[0] != null)
        {
            newVE = (bool)(inputs[0]);
        }

        bool iReset = default(bool);
        if (inputs[1] != null)
        {
            iReset = (bool)(inputs[1]);
        }

        bool iGo = default(bool);
        if (inputs[2] != null)
        {
            iGo = (bool)(inputs[2]);
        }

        int method = default(int);
        if (inputs[3] != null)
        {
            method = (int)(inputs[3]);
        }

        bool showBri = default(bool);
        if (inputs[4] != null)
        {
            showBri = (bool)(inputs[4]);
        }

        bool showCols = default(bool);
        if (inputs[5] != null)
        {
            showCols = (bool)(inputs[5]);
        }

        bool debug = default(bool);
        if (inputs[6] != null)
        {
            debug = (bool)(inputs[6]);
        }

        bool showPlns = default(bool);
        if (inputs[7] != null)
        {
            showPlns = (bool)(inputs[7]);
        }

        Box Box = default(Box);
        if (inputs[8] != null)
        {
            Box = (Box)(inputs[8]);
        }

        int Res = default(int);
        if (inputs[9] != null)
        {
            Res = (int)(inputs[9]);
        }

        List<System.Object> P = null;
        if (inputs[10] != null)
        {
            P = GH_DirtyCaster.CastToList<System.Object>(inputs[10]);
        }
        List<Box> food = null;
        if (inputs[11] != null)
        {
            food = GH_DirtyCaster.CastToList<Box>(inputs[11]);
        }
        Box obstacle = default(Box);
        if (inputs[12] != null)
        {
            obstacle = (Box)(inputs[12]);
        }

        int tL = default(int);
        if (inputs[13] != null)
        {
            tL = (int)(inputs[13]);
        }

        double nR = default(double);
        if (inputs[14] != null)
        {
            nR = (double)(inputs[14]);
        }

        double coS = default(double);
        if (inputs[15] != null)
        {
            coS = (double)(inputs[15]);
        }

        double alS = default(double);
        if (inputs[16] != null)
        {
            alS = (double)(inputs[16]);
        }

        double seS = default(double);
        if (inputs[17] != null)
        {
            seS = (double)(inputs[17]);
        }

        double seR = default(double);
        if (inputs[18] != null)
        {
            seR = (double)(inputs[18]);
        }

        double mForce = default(double);
        if (inputs[19] != null)
        {
            mForce = (double)(inputs[19]);
        }

        double sR = default(double);
        if (inputs[20] != null)
        {
            sR = (double)(inputs[20]);
        }

        double sI = default(double);
        if (inputs[21] != null)
        {
            sI = (double)(inputs[21]);
        }

        double dR = default(double);
        if (inputs[22] != null)
        {
            dR = (double)(inputs[22]);
        }

        double eR = default(double);
        if (inputs[23] != null)
        {
            eR = (double)(inputs[23]);
        }

        double sensDist = default(double);
        if (inputs[24] != null)
        {
            sensDist = (double)(inputs[24]);
        }

        double sensAng = default(double);
        if (inputs[25] != null)
        {
            sensAng = (double)(inputs[25]);
        }

        double stigS = default(double);
        if (inputs[26] != null)
        {
            stigS = (double)(inputs[26]);
        }



        //3. Declare output parameters
        object initColors = null;
        object points = null;
        object vectors = null;
        object oBri = null;
        object sensors = null;
        object planes = null;
        object trails = null;


        //4. Invoke RunScript
        RunScript(newVE, iReset, iGo, method, showBri, showCols, debug, showPlns, Box, Res, P, food, obstacle, tL, nR, coS, alS, seS, seR, mForce, sR, sI, dR, eR, sensDist, sensAng, stigS, ref initColors, ref points, ref vectors, ref oBri, ref sensors, ref planes, ref trails);

        try
        {
            //5. Assign output parameters to component...
            if (initColors != null)
            {
                if (GH_Format.TreatAsCollection(initColors))
                {
                    IEnumerable __enum_initColors = (IEnumerable)(initColors);
                    DA.SetDataList(1, __enum_initColors);
                }
                else
                {
                    if (initColors is Grasshopper.Kernel.Data.IGH_DataTree)
                    {
                        //merge tree
                        DA.SetDataTree(1, (Grasshopper.Kernel.Data.IGH_DataTree)(initColors));
                    }
                    else
                    {
                        //assign direct
                        DA.SetData(1, initColors);
                    }
                }
            }
            else
            {
                DA.SetData(1, null);
            }
            if (points != null)
            {
                if (GH_Format.TreatAsCollection(points))
                {
                    IEnumerable __enum_points = (IEnumerable)(points);
                    DA.SetDataList(2, __enum_points);
                }
                else
                {
                    if (points is Grasshopper.Kernel.Data.IGH_DataTree)
                    {
                        //merge tree
                        DA.SetDataTree(2, (Grasshopper.Kernel.Data.IGH_DataTree)(points));
                    }
                    else
                    {
                        //assign direct
                        DA.SetData(2, points);
                    }
                }
            }
            else
            {
                DA.SetData(2, null);
            }
            if (vectors != null)
            {
                if (GH_Format.TreatAsCollection(vectors))
                {
                    IEnumerable __enum_vectors = (IEnumerable)(vectors);
                    DA.SetDataList(3, __enum_vectors);
                }
                else
                {
                    if (vectors is Grasshopper.Kernel.Data.IGH_DataTree)
                    {
                        //merge tree
                        DA.SetDataTree(3, (Grasshopper.Kernel.Data.IGH_DataTree)(vectors));
                    }
                    else
                    {
                        //assign direct
                        DA.SetData(3, vectors);
                    }
                }
            }
            else
            {
                DA.SetData(3, null);
            }
            if (oBri != null)
            {
                if (GH_Format.TreatAsCollection(oBri))
                {
                    IEnumerable __enum_oBri = (IEnumerable)(oBri);
                    DA.SetDataList(4, __enum_oBri);
                }
                else
                {
                    if (oBri is Grasshopper.Kernel.Data.IGH_DataTree)
                    {
                        //merge tree
                        DA.SetDataTree(4, (Grasshopper.Kernel.Data.IGH_DataTree)(oBri));
                    }
                    else
                    {
                        //assign direct
                        DA.SetData(4, oBri);
                    }
                }
            }
            else
            {
                DA.SetData(4, null);
            }
            if (sensors != null)
            {
                if (GH_Format.TreatAsCollection(sensors))
                {
                    IEnumerable __enum_sensors = (IEnumerable)(sensors);
                    DA.SetDataList(5, __enum_sensors);
                }
                else
                {
                    if (sensors is Grasshopper.Kernel.Data.IGH_DataTree)
                    {
                        //merge tree
                        DA.SetDataTree(5, (Grasshopper.Kernel.Data.IGH_DataTree)(sensors));
                    }
                    else
                    {
                        //assign direct
                        DA.SetData(5, sensors);
                    }
                }
            }
            else
            {
                DA.SetData(5, null);
            }
            if (planes != null)
            {
                if (GH_Format.TreatAsCollection(planes))
                {
                    IEnumerable __enum_planes = (IEnumerable)(planes);
                    DA.SetDataList(6, __enum_planes);
                }
                else
                {
                    if (planes is Grasshopper.Kernel.Data.IGH_DataTree)
                    {
                        //merge tree
                        DA.SetDataTree(6, (Grasshopper.Kernel.Data.IGH_DataTree)(planes));
                    }
                    else
                    {
                        //assign direct
                        DA.SetData(6, planes);
                    }
                }
            }
            else
            {
                DA.SetData(6, null);
            }
            if (trails != null)
            {
                if (GH_Format.TreatAsCollection(trails))
                {
                    IEnumerable __enum_trails = (IEnumerable)(trails);
                    DA.SetDataList(7, __enum_trails);
                }
                else
                {
                    if (trails is Grasshopper.Kernel.Data.IGH_DataTree)
                    {
                        //merge tree
                        DA.SetDataTree(7, (Grasshopper.Kernel.Data.IGH_DataTree)(trails));
                    }
                    else
                    {
                        //assign direct
                        DA.SetData(7, trails);
                    }
                }
            }
            else
            {
                DA.SetData(7, null);
            }

        }
        catch (Exception ex)
        {
            this.__err.Add(string.Format("Script exception: {0}", ex.Message));
        }
        finally
        {
            //Add errors and messages... 
            if (owner.Params.Output.Count > 0)
            {
                if (owner.Params.Output[0] is Grasshopper.Kernel.Parameters.Param_String)
                {
                    List<string> __errors_plus_messages = new List<string>();
                    if (this.__err != null) { __errors_plus_messages.AddRange(this.__err); }
                    if (this.__out != null) { __errors_plus_messages.AddRange(this.__out); }
                    if (__errors_plus_messages.Count > 0)
                        DA.SetDataList(0, __errors_plus_messages);
                }
            }
        }
    }
}
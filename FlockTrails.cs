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
using Rhino.Render;
using System.Diagnostics;
// </Custom using>


/// <summary>
/// This class will be instantiated on demand by the Script component.
/// </summary>
public class Script_Instance2 : GH_ScriptInstance
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
    private void RunScript(bool iReset, bool iGo, bool showPoints, bool showPlanes, bool showFrames, bool showValues, List<System.Object> P, List<System.Object> trails, double nR, double maxForce, double alFrame, double sepUp, double sepDown, double surfCoUp, double surfCoDown, double trailCoUp, double trailCoDown, double seR, ref object points, ref object planes, ref object frames, ref object TrailCIntensity, ref object SurfCIntensity, ref object SepIntensity)
    {

        // <Custom code>

        if (P == null) return;


        List<NurbsCurve> trailsList = trails.Select(p => (NurbsCurve)p).ToList();
        List<Point3d> pointsList = P.Select(p => (Point3d)p).ToList();

        if (APS == null)
        {
            APS = new AgentPlaneSystem(pointsList, trailsList);
        }

        if (iReset)
        {
            APS = new AgentPlaneSystem(pointsList, trailsList);
            c = 0;
        }

        if (iGo)
        {
            c++;

            if (c < 50) APS.mForce = 1.0;
            else if (c > 50 & c<100) APS.mForce = 0.7;
            else if (c > 100 & c < 150) APS.mForce = 0.4;
            else if (c > 150 & c < 200) APS.mForce = 0.1;
            //APS.mForce = maxForce;
            APS.NeighborhoodRadius = nR;
            APS.AlignmentStrengthFrame = alFrame;
            APS.separationUp = sepUp;
            APS.separationDown = sepDown;
            APS.surfCohesionUp = surfCoUp;
            APS.surfCohesionDown = surfCoDown;
            if (c < 200)
            {
                APS.trailCohesionUp = trailCoUp;
                APS.trailCohesionDown = trailCoDown;
            }
            else if (c > 200)
            {
                APS.trailCohesionUp = 0.0;
                APS.trailCohesionDown = 0.0;
            }
            APS.SeparationRadius = seR;

            APS.Define();
            APS.Update();

            if (c < 220)
                Component.ExpireSolution(true);
            else
                iGo = false;

        }

        if (showPoints) points = APS.GetPoints();
        if (showPlanes) planes = APS.GetPlanes();
        if (showFrames) frames = APS.GetFrames();

        if (showValues)
        {
            TrailCIntensity = APS.GetRemappedTrailCohesion();
            SurfCIntensity = APS.GetRemappedSurfCohesion();
            SepIntensity = APS.GetRemappedSeparation();
        }

        Print(c.ToString());

        // </Custom code>
    }

    // <Custom additional code> 

    public AgentPlaneSystem APS;
    public int c;

    /********************************************************************************************/
    public class AgentPlaneSystem
    {
        public List<AgentPlane> AgentPlanes;
        public List<NurbsCurve> StigTrails;
        public double NeighborhoodRadius;
        public double NeighborhoodSurfRadius;
        public double CohesionStrength;
        public double AlignmentStrengthFrame;
        public double AlignmentStrength;
        public double separationUp;
        public double separationDown;
        public double surfCohesionUp;
        public double surfCohesionDown;
        public double trailCohesionUp;
        public double trailCohesionDown;
        public double SeparationRadius;
        public double sepDistSquared;
        public double MaxSpeed;
        public double mForce;
        public double ContainmentStrength;
        RTree PointsRTree;

        public AgentPlaneSystem(List<Point3d> P, List<NurbsCurve> StigTrails)
        {
            this.StigTrails = StigTrails;
            AgentPlanes = new List<AgentPlane>();

            for (int i = 0; i < P.Count; i++) AgentPlanes.Add(new AgentPlane(this, P[i], RandomVector(P, 2)[i]));

            MaxSpeed = 0.5;
        }

        //__________________________________________________//
        public void Update()
        {
            PointsRTree = new RTree();
            for (int i = 0; i < AgentPlanes.Count; i++)
                PointsRTree.Insert(AgentPlanes[i].O, i);

            List<AgentPlane> neighbours;

            foreach (AgentPlane agent in AgentPlanes)
            {
                neighbours = new List<AgentPlane>();

                EventHandler<RTreeEventArgs> rTreeCallBack = (object sender, RTreeEventArgs args) =>
                {
                    if (AgentPlanes[args.Id] != agent)
                        neighbours.Add(AgentPlanes[args.Id]);
                };

                PointsRTree.Search(new Sphere(agent.O, NeighborhoodRadius), rTreeCallBack);


                agent.ComputeDesiredNeighbours(neighbours);
            }

            Parallel.ForEach(AgentPlanes, ag =>
            {
                ag.FlockClosestPoint(ag.closPoint);
                ag.Update();
                ag.ResetDesireds();
            });
        }

        public void Define()
        {
            sepDistSquared = 2 * SeparationRadius * SeparationRadius;

            Parallel.ForEach(AgentPlanes, ag =>
            {
                Point3d testPoint = new Point3d();
                double testDistance = Double.MaxValue;

                for (int i = 0; i < StigTrails.Count; i++)
                {
                    double closParam;
                    bool successCP = StigTrails[i].ClosestPoint(ag.O, out closParam);

                    if (successCP) testPoint = StigTrails[i].PointAt(closParam);

                    ag.distSquared = (testPoint - ag.O).Length;

                    if (ag.distSquared < testDistance)
                    {
                        testDistance = ag.distSquared;
                        ag.closPoint = new Point3d(testPoint);

                        Plane testPlane;
                        bool successF = StigTrails[i].PerpendicularFrameAt(closParam, out testPlane);
                        if (successF) ag.frame = testPlane;

                        ag.distSquared = 0.0;
                    }
                }

            });
        }

        public GH_Point[] GetPoints()
        {
            GH_Point[] pts = new GH_Point[AgentPlanes.Count];

            Parallel.For(0, AgentPlanes.Count, i =>
            {
                pts[i] = new GH_Point(AgentPlanes[i].O);
            });
            return pts;
        }

        public GH_Plane[] GetPlanes()
        {
            GH_Plane[] gP = new GH_Plane[AgentPlanes.Count];
            Parallel.For(0, AgentPlanes.Count, i =>
            {
                gP[i] = new GH_Plane(AgentPlanes[i].plane);
            });
            return gP;
        }

        public Plane[] GetFrames()
        {
            Plane[] frames = new Plane[AgentPlanes.Count];
            Parallel.For(0, AgentPlanes.Count, i =>
            {
                frames[i] = AgentPlanes[i].frame;
            });
            return frames;
        }

        public double[] GetDistSquared()
        {
            double[] dS = new double[AgentPlanes.Count];
            Parallel.For(0, AgentPlanes.Count, i =>
            {
                dS[i] = AgentPlanes[i].distSquared;
            });
            return dS;
        }

        public float[] GetRemappedTrailCohesion()
        {
            float[] rV = new float[AgentPlanes.Count];
            Parallel.For(0, AgentPlanes.Count, i =>
            {
                rV[i] = (float)AgentPlanes[i].remappedForTrailCohesion;
            });
            return rV;
        }

        public float[] GetRemappedSurfCohesion()
        {
            float[] rV = new float[AgentPlanes.Count];
            Parallel.For(0, AgentPlanes.Count, i =>
            {
                rV[i] = (float)AgentPlanes[i].remappedForSurfCohesion;
            });
            return rV;
        }

        public float[] GetRemappedSeparation()
        {
            float[] rV = new float[AgentPlanes.Count];
            Parallel.For(0, AgentPlanes.Count, i =>
            {
                rV[i] = (float)AgentPlanes[i].remappedForSeparation;
            });
            return rV;
        }

    }
    public class AgentPlane
    {
        public AgentPlaneSystem pSystem;
        public Point3d O;
        public Plane plane;
        public Vector3d X;
        public Vector3d Y;
        public Vector3d Z;
        public Vector3d desiredVel, desiredX, desiredY, desiredZ;
        public Vector3d vel;
        public Point3d closPoint;
        public Plane frame;
        public double distSquared;
        public double remappedForTrailCohesion;
        public double remappedForSurfCohesion;
        public double remappedForSeparation;

        public AgentPlane(AgentPlaneSystem pSystem, Point3d O, Vector3d X)
        {
            this.pSystem = pSystem;
            this.O = O;
            this.X = X;
            X.Unitize();
            Vector3d oV = new Vector3d(this.O);
            oV.Unitize();
            Y = Vector3d.CrossProduct(oV, this.X);
            Z = Vector3d.CrossProduct(X, Y);
        }

        public void Update()
        {
            vel = 0.8 * vel + 0.2 * desiredVel;
            if (vel.Length > pSystem.MaxSpeed)
            {
                vel.Unitize();
                vel *= pSystem.MaxSpeed;
            }

            O += vel * pSystem.mForce;

            X = 0.9 * X + 0.1 * desiredX;
            X.Unitize();

            Z = 0.9 * Z + 0.1 * desiredZ;
            Z.Unitize();

            plane = ExtractPlane();
        }

        public void ResetDesireds()
        {
            desiredVel = Vector3d.Zero;
            desiredX = Vector3d.Zero;
            desiredZ = Vector3d.Zero;
        }

        public void ComputeDesiredNeighbours(List<AgentPlane> neighbours)
        {
            // neighbours interaction
            if (neighbours.Count != 0)
            {
                // define vectors
                Vector3d alignX = Vector3d.Zero;
                Vector3d alignZ = Vector3d.Zero;
                Vector3d separation = Vector3d.Zero;
                Vector3d surfCohesion = Vector3d.Zero;
                Vector3d sepPointer;
                Point3d target;

                int sepCount = 0;
                double distanceSquared;
                double invNCount = 1.0 / neighbours.Count;


                foreach (AgentPlane neighbour in neighbours)
                {
                    //alignment with frame condition
                    if (Vector3d.VectorAngle(X, neighbour.frame.ZAxis) < Math.PI * 0.5)
                        alignX += neighbour.frame.ZAxis;
                    else alignX -= neighbour.frame.ZAxis;

                    if (Vector3d.VectorAngle(Z, neighbour.frame.YAxis) < Math.PI * 0.5)
                        alignZ += neighbour.frame.YAxis;
                    else alignZ -= neighbour.frame.YAxis;

                    //alignment with neighbours condition
                    if (Vector3d.VectorAngle(X, neighbour.X * invNCount) < Math.PI * 0.5)
                        alignX += neighbour.X;
                    else alignX -= neighbour.X;

                    if (Vector3d.VectorAngle(Z, neighbour.Z * invNCount) < Math.PI * 0.5)
                        alignZ += neighbour.Z;
                    else alignZ -= neighbour.Z;


                    sepPointer = O - neighbour.O;
                    distanceSquared = O.DistanceToSquared(neighbour.O);

                    if (distanceSquared < pSystem.sepDistSquared)
                    {
                        separation += sepPointer /= (sepPointer.Length * distanceSquared);
                        sepCount++;
                    }


                    Line testLine = new Line(O, Z);
                    target = testLine.ClosestPoint(neighbour.O, false);
                    surfCohesion += (target - O);

                }

                //alignment behavior

                alignX *= invNCount;
                alignZ *= invNCount;

                desiredX += pSystem.AlignmentStrengthFrame * alignX;
                desiredZ += pSystem.AlignmentStrengthFrame * alignZ;

                //separation behavior

                separation.Transform(Transform.PlanarProjection(ExtractPlane()));

                if (sepCount > 0) separation /= sepCount;

                remappedForSeparation = Map(distSquared, pSystem.GetDistSquared().Min(), pSystem.GetDistSquared().Max(), pSystem.separationUp, pSystem.separationDown);
                desiredVel += remappedForSeparation * separation;

                //surfCohesion behavior

                surfCohesion *= invNCount;
                remappedForSurfCohesion = Map(distSquared, pSystem.GetDistSquared().Min(), pSystem.GetDistSquared().Max(), pSystem.surfCohesionUp, pSystem.surfCohesionDown);
                desiredVel += remappedForSurfCohesion * surfCohesion;

            }

        }

        public void FlockClosestPoint(Point3d target)
        {
            Vector3d cohesion = target - O;
            remappedForTrailCohesion = Map(distSquared, pSystem.GetDistSquared().Min(), pSystem.GetDistSquared().Max(), pSystem.trailCohesionUp, pSystem.trailCohesionDown);
            cohesion.Unitize();
            cohesion *= remappedForTrailCohesion;
            desiredVel += cohesion;
        }

        public Plane ExtractPlane()
        {
            return new Plane(O, X, -(Vector3d.CrossProduct(X, Z)));
        }

    }



    #region Utilities

    public static Vector3d VectorAmp(Vector3d v, double Amplitude)
    {
        v.Unitize();
        return (v * Amplitude);
    }

    public static List<Vector3d> RandomVector(List<Point3d> pts, int seed)
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

    public static double Map(double x, double in_min, double in_max, double out_min, double out_max)
    {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
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
            return inclusiveMinimum;
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
        bool iReset = default(bool);
        if (inputs[0] != null)
        {
            iReset = (bool)(inputs[0]);
        }

        bool iGo = default(bool);
        if (inputs[1] != null)
        {
            iGo = (bool)(inputs[1]);
        }

        bool showPlanes = default(bool);
        if (inputs[2] != null)
        {
            showPlanes = (bool)(inputs[2]);
        }

        Box Box = default(Box);
        if (inputs[3] != null)
        {
            Box = (Box)(inputs[3]);
        }

        int Res = default(int);
        if (inputs[4] != null)
        {
            Res = (int)(inputs[4]);
        }

        List<System.Object> P = null;
        if (inputs[5] != null)
        {
            P = GH_DirtyCaster.CastToList<System.Object>(inputs[5]);
        }
        List<Polyline> trails = null;
        if (inputs[6] != null)
        {
            trails = GH_DirtyCaster.CastToList<Polyline>(inputs[6]);
        }
        double nR = default(double);
        if (inputs[7] != null)
        {
            nR = (double)(inputs[7]);
        }

        double coS = default(double);
        if (inputs[8] != null)
        {
            coS = (double)(inputs[8]);
        }

        double alP = default(double);
        if (inputs[9] != null)
        {
            alP = (double)(inputs[9]);
        }

        double alS = default(double);
        if (inputs[10] != null)
        {
            alS = (double)(inputs[10]);
        }

        double seS = default(double);
        if (inputs[11] != null)
        {
            seS = (double)(inputs[11]);
        }

        double seR = default(double);
        if (inputs[12] != null)
        {
            seR = (double)(inputs[12]);
        }

        double mForce = default(double);
        if (inputs[13] != null)
        {
            mForce = (double)(inputs[13]);
        }



        //3. Declare output parameters
        object points = null;
        object vectors = null;
        object planes = null;


        //4. Invoke RunScript
        RunScript(iReset, iGo, showPlanes, Box, Res, P, trails, nR, coS, alP, alS, seS, seR, mForce, ref points, ref vectors, ref planes);

        try
        {
            //5. Assign output parameters to component...
            if (points != null)
            {
                if (GH_Format.TreatAsCollection(points))
                {
                    IEnumerable __enum_points = (IEnumerable)(points);
                    DA.SetDataList(1, __enum_points);
                }
                else
                {
                    if (points is Grasshopper.Kernel.Data.IGH_DataTree)
                    {
                        //merge tree
                        DA.SetDataTree(1, (Grasshopper.Kernel.Data.IGH_DataTree)(points));
                    }
                    else
                    {
                        //assign direct
                        DA.SetData(1, points);
                    }
                }
            }
            else
            {
                DA.SetData(1, null);
            }
            if (vectors != null)
            {
                if (GH_Format.TreatAsCollection(vectors))
                {
                    IEnumerable __enum_vectors = (IEnumerable)(vectors);
                    DA.SetDataList(2, __enum_vectors);
                }
                else
                {
                    if (vectors is Grasshopper.Kernel.Data.IGH_DataTree)
                    {
                        //merge tree
                        DA.SetDataTree(2, (Grasshopper.Kernel.Data.IGH_DataTree)(vectors));
                    }
                    else
                    {
                        //assign direct
                        DA.SetData(2, vectors);
                    }
                }
            }
            else
            {
                DA.SetData(2, null);
            }
            if (planes != null)
            {
                if (GH_Format.TreatAsCollection(planes))
                {
                    IEnumerable __enum_planes = (IEnumerable)(planes);
                    DA.SetDataList(3, __enum_planes);
                }
                else
                {
                    if (planes is Grasshopper.Kernel.Data.IGH_DataTree)
                    {
                        //merge tree
                        DA.SetDataTree(3, (Grasshopper.Kernel.Data.IGH_DataTree)(planes));
                    }
                    else
                    {
                        //assign direct
                        DA.SetData(3, planes);
                    }
                }
            }
            else
            {
                DA.SetData(3, null);
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
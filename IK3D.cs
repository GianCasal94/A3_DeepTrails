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
using System.Threading.Tasks;
using System.Linq;
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
    private void RunScript(bool iReset, bool iGo, Mesh M, List<System.Object> Pl, List<Polyline> body, double sR, double minDist, double aVision, ref object Bodies, ref object cBodies, ref object sF)
    {
        // <Custom code>

        List<Plane> plList = Pl.Select(p => (Plane)p).ToList();


        if (iReset || ABS == null)
        {
            ABS = new AgentBodySystem(plList, body, sR, minDist, aVision);
            iterationsCount = 0;
        }

        if (iGo)
        {

            ABS.Update();
            iterationsCount++;
            Component.ExpireSolution(true);

        }



        //extract geometries
        DataTree<Polyline> outBodies = new DataTree<Polyline>();

        for (int i = 0; i < ABS.AgentBodies.Length; i++)
            outBodies.AddRange(ABS.AgentBodies[i].ExtractBody(), new GH_Path(i));

        Bodies = outBodies;
        cBodies = ABS.GetCurveBodies();

        Print(iterationsCount.ToString());
        // </Custom code>
    }

    // <Custom additional code> 


    AgentBodySystem ABS;
    public int iterationsCount;
    public class AgentBodySystem
    {
        public AgentBody[] AgentBodies;
        public double searchRadius;
        public RTree PointsRTree;
        public double minDist;
        public double aVision;

        public AgentBodySystem(List<Plane> Pl, List<Polyline> body, double searchRadius, double minDist, double aVision)
        {
            AgentBodies = new AgentBody[Pl.Count];
            PointsRTree = new RTree();
            this.searchRadius = searchRadius;
            this.minDist = minDist;
            this.aVision = aVision;

            for (int i = 0; i < Pl.Count; i++)
            {
                AgentBodies[i] = new AgentBody(Pl[i], body, minDist, aVision);
                PointsRTree.Insert(AgentBodies[i].body.O, i);
            }
        }

        public void Update()
        {
            foreach (AgentBody ab in AgentBodies)
                PointsRTree.Search(new Sphere(ab.body.O, searchRadius), (sender, args) =>
                {
                    if (AgentBodies[args.Id] != ab) ab.Neighbours.Add(AgentBodies[args.Id]);
                });

            Parallel.ForEach(AgentBodies, ab =>
            {
                ab.FindNeighTip();
                ab.Update();
            });
        }


        public DataTree<Curve> GetCurveBodies()
        {

            DataTree<Curve> allBodies = new DataTree<Curve>();

            for (int i = 0; i < AgentBodies.Length; i++)
            {
                GH_Path path = new GH_Path(i);
                for (int j = 0; j < AgentBodies[i].GetCurveBody().AllData().Count; j++)
                    allBodies.Add((AgentBodies[i].GetCurveBody().AllData()[j]), path);
            }
            return allBodies;
        }

    }

    public class AgentBody
    {
        public Plane agentPlane;
        public Body body;
        public double minDist;
        public List<AgentBody> Neighbours;
        public List<Curve> cBody;
        public double aVision;
        public Mesh M;

        public AgentBody(Plane agentPlane, List<Polyline> polylines, double minDist, double aVision)
        {

            this.agentPlane = agentPlane;
            this.minDist = minDist;
            this.aVision = aVision;
            body = new Body(polylines, minDist);
            body.Orient(Plane.WorldXY, this.agentPlane);

            Neighbours = new List<AgentBody>();
        }

        public void Update()
        {
            foreach (Tentacle tent in body.tentacles)
            {
                if (tent.segments[tent.segments.Length - 1].b.DistanceTo(tent.target) < body.minDist & Vector3d.VectorAngle((tent.target - tent.segments[tent.segments.Length - 1].b), (tent.segments[tent.segments.Length - 1].b - tent.segments[tent.segments.Length - 1].a)) < aVision)
                {
                    tent.Update();

                    if (tent.segments[tent.total - 1].b.DistanceTo(tent.target) > Double.MinValue)
                    {
                        tent.segments[tent.segments.Length - 1].b = tent.target;
                    }

                }

            }
        }

        public void FindNeighTip()
        {
            double distSquaredFinal;
            double distSquaredPrev;
            double minD;
            foreach (Tentacle tent in body.tentacles)
            {
                minD = Double.MaxValue;

                foreach (AgentBody neighBody in Neighbours)
                {
                    foreach (Tentacle neighTent in neighBody.body.tentacles)
                    {

                        distSquaredFinal = tent.segments[tent.total - 1].b.DistanceTo(neighTent.segments[tent.total - 1].b);
                        distSquaredPrev = tent.segments[tent.total - 1].b.DistanceTo(neighTent.segments[tent.total - 2].b);

                        if (distSquaredFinal < minD || distSquaredPrev < minD)
                        {
                            if (distSquaredFinal < distSquaredPrev)
                            {
                                minD = distSquaredFinal;
                                tent.nearestTentacle = neighTent;
                                Line line = new Line(tent.segments[tent.total - 1].b, tent.nearestTentacle.segments[tent.nearestTentacle.total - 1].b);
                                tent.target = line.PointAt(0.5);
                            }
                            else
                            {
                                minD = distSquaredPrev;
                                tent.nearestTentacle = neighTent;
                                tent.target = tent.nearestTentacle.segments[tent.total - 2].b;
                            }
                        }
                    }
                }
            }
        }

        public List<Polyline> ExtractBody()
        {
            body.Rebuild();
            return body.Arms;
        }

        public DataTree<Curve> GetCurveBody()
        {
            DataTree<Curve> allTentacles = new DataTree<Curve>();

            for (int i = 0; i < body.tentacles.Length; i++)
            {
                GH_Path path = new GH_Path(i);
                allTentacles.Add((body.tentacles[i].BuildCurvedTentacle()), path);
            }
            return allTentacles;
        }
    }

    public class Body
    {
        public List<Polyline> Arms;
        public List<Curve> curvedArms;
        public Tentacle[] tentacles;
        public Point3d O;
        public double minDist;
        public Body(List<Polyline> polylines, double minDist)
        {
            this.minDist = minDist;

            Arms = new List<Polyline>();
            foreach (Polyline p in polylines)
                Arms.Add(p.Duplicate());

            O = Arms[0][0];

            tentacles = new Tentacle[polylines.Count];

            for (int i = 0; i < Arms.Count; i++)
            {
                tentacles[i] = new Tentacle(this, Arms[i], Math.Atan2((Arms[i][1] - Arms[i][0]).Y, (Arms[i][1] - Arms[i][0]).X), Math.Atan2((Arms[i][2] - Arms[i][1]).Y, (Arms[i][2] - Arms[i][1]).X), 0.0, 0.0);
            }
        }

        public void Orient(Plane oldPlane, Plane newPlane)
        {
            var x = Transform.PlaneToPlane(oldPlane, newPlane);

            O.Transform(x);

            for (int i = 0; i < Arms.Count; i++)
            {
                Arms[i].Transform(x);

                if (Arms[i].Count == 3)
                {
                    tentacles[i].segments[tentacles[i].total - 2].a = Arms[i][0];
                    tentacles[i].segments[tentacles[i].total - 1].a = Arms[i][1];
                    tentacles[i].segments[tentacles[i].total - 1].b = Arms[i][2];
                }
                if (Arms[i].Count == 4)
                {
                    tentacles[i].segments[tentacles[i].total - 3].a = Arms[i][0];
                    tentacles[i].segments[tentacles[i].total - 2].a = Arms[i][1];
                    tentacles[i].segments[tentacles[i].total - 1].a = Arms[i][2];
                    tentacles[i].segments[tentacles[i].total - 1].b = Arms[i][3];
                }
                if (Arms[i].Count == 5)
                {
                    tentacles[i].segments[tentacles[i].total - 4].a = Arms[i][0];
                    tentacles[i].segments[tentacles[i].total - 3].a = Arms[i][1];
                    tentacles[i].segments[tentacles[i].total - 2].a = Arms[i][2];
                    tentacles[i].segments[tentacles[i].total - 1].a = Arms[i][3];
                    tentacles[i].segments[tentacles[i].total - 1].b = Arms[i][4];
                }
            }


        }

        public void Rebuild()
        {
            for (int i = 0; i < Arms.Count; i++)
            {
                tentacles[i].RebuildTentacle();
                Arms[i] = tentacles[i].polyTent;
            }
        }

    }

    public class Tentacle
    {
        public Segment[] segments;
        public int total;
        public Body body;
        public Point3d target;
        public Polyline polyTent;
        public Tentacle nearestTentacle;
        public double firstAngle1;
        public double secondAngle1;
        public double firstAngle2;
        public double secondAngle2;
        List<Point3d> controlPoints;
        public Tentacle(Body body, Polyline polyline, double firstAngle1, double secondAngle1, double firstAngle2, double secondAngle2)
        {
            this.body = body;
            this.polyTent = polyline;
            this.firstAngle1 = firstAngle1;
            this.secondAngle1 = secondAngle1;
            this.firstAngle2 = firstAngle2;
            this.secondAngle2 = secondAngle2;
            segments = new Segment[polyTent.Count - 1];

            total = segments.Length;

            segments[0] = new Segment(new Line(polyline[0], polyline[1]), firstAngle1, firstAngle2);

            for (int i = 1; i < segments.Length; i++)
            {
                segments[i] = new Segment(new Line(polyline[i], polyline[i + 1]), segments[i - 1], secondAngle1, secondAngle2);
            }
        }


        public void Update()
        {

            segments[total - 1].Follow(target);
            segments[total - 1].Update();


            for (int i = total - 2; i >= 0; i--)
            {
                segments[i].Follow(segments[i + 1]);
                segments[i].Update();
            }

            segments[0].SetA(polyTent[0]);

            for (int i = 1; i < total; i++)
            {
                segments[i].SetA(segments[i - 1].b);
            }

        }

        public void RebuildTentacle()
        {
            for (int i = 0; i < polyTent.Count - 1; i++)
                polyTent[i] = segments[i].a;

            polyTent[polyTent.Count - 1] = segments[total - 1].b;

        }

        public Curve BuildCurvedTentacle()
        {
            RebuildTentacle();

            Curve tentacle;

          
            if (polyTent[polyTent.Count - 1].DistanceToSquared(target) < 0.01)
            {
                if (target.DistanceToSquared(nearestTentacle.polyTent[polyTent.Count - 1]) < 0.01)
                {
                    controlPoints = new List<Point3d>();
                    for (int i = 0; i < polyTent.Count - 1; i++)
                        controlPoints.Add(polyTent[i]);
                    for (int i = nearestTentacle.polyTent.Count - 2; i >= 0; i--)
                        controlPoints.Add(nearestTentacle.polyTent[i]);
                }
                else
                {
                    controlPoints = new List<Point3d>();
                    for (int i = 0; i < polyTent.Count - 1; i++)
                        controlPoints.Add(polyTent[i]);
                    for (int i = nearestTentacle.polyTent.Count - 2; i >= 0; i--)
                        controlPoints.Add(nearestTentacle.polyTent[i]);
                }
            }
            else
            {
                controlPoints = new List<Point3d>();
                for (int i = 0; i < polyTent.Count; i++)
                    controlPoints.Add(polyTent[i]);
            }


            tentacle = Curve.CreateInterpolatedCurve(controlPoints, 3);

            return tentacle;

        }
    }


    public class Segment
    {
        public Line segment;
        public Point3d a;
        public Point3d b;
        public double len;
        public double hAngle;
        public double vAngle;
        public Segment(Line segment, double hAngle, double vAngle)
        {
            this.segment = segment;
            a = segment.PointAtLength(0.0);
            len = segment.Length;
            this.hAngle = hAngle;
            this.vAngle = vAngle;
            CalculateB();
        }

        public Segment(Line segment, Segment parent, double hAngle, double vAngle)
        {
            this.segment = segment;
            a = new Point3d(parent.b);
            len = segment.Length;
            this.hAngle = hAngle;
            this.vAngle = vAngle;
            CalculateB();
        }

        public void CalculateB()
        {
            double dx = len * Math.Sin(vAngle) * Math.Cos(hAngle);
            double dy = len * Math.Sin(hAngle) * Math.Sin(vAngle);
            double dz = len * Math.Cos(vAngle);
            b = new Point3d(a.X + dx, a.Y + dy, a.Z + dz);
        }

        public void Follow(Segment child)
        {
            Point3d target = child.a;
            Follow(target);
        }

        public void Follow(Point3d target)
        {
            //Define direction
            Vector3d direction = target - a;

            //Calculate angles
            hAngle = Math.Atan2(direction.Y, direction.X);
            vAngle = Math.Acos(direction.Z / (Math.Sqrt((direction.X * direction.X) + (direction.Y * direction.Y) + (direction.Z * direction.Z))));

            //Set magnitude of direction
            direction.Unitize();
            direction *= -len;

            //Upload a
            a = new Point3d(target + direction);
        }

        public Line ExtractSegment()
        {
            return new Line(a, b);
        }

        public void SetA(Point3d pos)
        {
            a = new Point3d(pos);
            CalculateB();
        }

        public void Update()
        {
            CalculateB();
        }
    }

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

        List<System.Object> Pl = null;
        if (inputs[2] != null)
        {
            Pl = GH_DirtyCaster.CastToList<System.Object>(inputs[2]);
        }
        double sR = default(double);
        if (inputs[3] != null)
        {
            sR = (double)(inputs[3]);
        }

        double minDist = default(double);
        if (inputs[4] != null)
        {
            minDist = (double)(inputs[4]);
        }

        double bodyScale = default(double);
        if (inputs[5] != null)
        {
            bodyScale = (double)(inputs[5]);
        }



        //3. Declare output parameters
        object Bodies = null;
        object cBodies = null;


        //4. Invoke RunScript
        RunScript(iReset, iGo, Pl, sR, minDist, bodyScale, ref Bodies, ref cBodies);

        try
        {
            //5. Assign output parameters to component...
            if (Bodies != null)
            {
                if (GH_Format.TreatAsCollection(Bodies))
                {
                    IEnumerable __enum_Bodies = (IEnumerable)(Bodies);
                    DA.SetDataList(1, __enum_Bodies);
                }
                else
                {
                    if (Bodies is Grasshopper.Kernel.Data.IGH_DataTree)
                    {
                        //merge tree
                        DA.SetDataTree(1, (Grasshopper.Kernel.Data.IGH_DataTree)(Bodies));
                    }
                    else
                    {
                        //assign direct
                        DA.SetData(1, Bodies);
                    }
                }
            }
            else
            {
                DA.SetData(1, null);
            }
            if (cBodies != null)
            {
                if (GH_Format.TreatAsCollection(cBodies))
                {
                    IEnumerable __enum_cBodies = (IEnumerable)(cBodies);
                    DA.SetDataList(2, __enum_cBodies);
                }
                else
                {
                    if (cBodies is Grasshopper.Kernel.Data.IGH_DataTree)
                    {
                        //merge tree
                        DA.SetDataTree(2, (Grasshopper.Kernel.Data.IGH_DataTree)(cBodies));
                    }
                    else
                    {
                        //assign direct
                        DA.SetData(2, cBodies);
                    }
                }
            }
            else
            {
                DA.SetData(2, null);
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
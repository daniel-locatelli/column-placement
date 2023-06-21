using System;
using System.Collections;
using System.Collections.Generic;

using Rhino;
using Rhino.Geometry;

using Grasshopper;
using Grasshopper.Kernel;
using Grasshopper.Kernel.Data;
using Grasshopper.Kernel.Types;

using System.IO;
using System.Linq;
using System.Data;
using System.Drawing;
using System.Reflection;
using System.Windows;
using System.Xml;
using System.Xml.Linq;
using System.Runtime.InteropServices;
using Rhino.DocObjects;
using Rhino.Collections;
using GH_IO;
using GH_IO.Serialization;
using KangarooSolver;
using KangarooSolver.Goals;
using Rhino.Render.ChangeQueue;
using Rhino.Runtime.InProcess;
using Rhino.Render;


/// <summary>
/// This class will be instantiated on demand by the Script component.
/// </summary>
public abstract class Script_Instance_6ed5d : GH_ScriptInstance
{
  #region Utility functions
  /// <summary>Print a String to the [Out] Parameter of the Script component.</summary>
  /// <param name="text">String to print.</param>
  private void Print(string text) { /* Implementation hidden. */ }
  /// <summary>Print a formatted String to the [Out] Parameter of the Script component.</summary>
  /// <param name="format">String format.</param>
  /// <param name="args">Formatting parameters.</param>
  private void Print(string format, params object[] args) { /* Implementation hidden. */ }
  /// <summary>Print useful information about an object instance to the [Out] Parameter of the Script component. </summary>
  /// <param name="obj">Object instance to parse.</param>
  private void Reflect(object obj) { /* Implementation hidden. */ }
  /// <summary>Print the signatures of all the overloads of a specific method to the [Out] Parameter of the Script component. </summary>
  /// <param name="obj">Object instance to parse.</param>
  private void Reflect(object obj, string method_name) { /* Implementation hidden. */ }
  #endregion

  #region Members
  /// <summary>Gets the current Rhino document.</summary>
  private readonly RhinoDoc RhinoDocument;
  /// <summary>Gets the Grasshopper document that owns this script.</summary>
  private readonly GH_Document GrasshopperDocument;
  /// <summary>Gets the Grasshopper script component that owns this script.</summary>
  private readonly IGH_Component Component;
  /// <summary>
  /// Gets the current iteration count. The first call to RunScript() is associated with Iteration==0.
  /// Any subsequent call within the same solution will increment the Iteration count.
  /// </summary>
  private readonly int Iteration;
  #endregion
  /// <summary>
  /// This procedure contains the user code. Input parameters are provided as regular arguments,
  /// Output parameters as ref arguments. You don't have to assign output parameters,
  /// they will have a default value.
  /// </summary>
  #region Runscript
  private void RunScript(Curve boundary, double offset, List<Curve> negative, List<Point3d> fixedColumns, int numberColumns, List<Curve> fixedWalls, double radius, bool reset, bool run, ref object points, ref object iterations)
  {
    /*
        //add the goals to the solver
        foreach (IGoal goal in allGoals)
        {
          PS.AssignPIndex(goal, 0.0001);
          Print(goal.ToString());
        }
    */

    if (reset || PS.ParticleCount() > numberColumns)
    {
      PS.ClearParticles();
      allGoals.Clear();
      counter = 0;

      //get brep boundary
      Brep brepBoundary = getBoundary(offset, boundary, negative);

      //get columns
      allColumns = getColumns(brepBoundary, numberColumns, fixedColumns);

      //convert brep to mesh
      Rhino.Geometry.Mesh meshBoundary = Rhino.Geometry.Mesh.CreateFromBrep(brepBoundary, MeshingParameters.Default)[0];

      //get goals
      allGoals = getGoals(allColumns, fixedColumns, fixedWalls, meshBoundary, radius);

      foreach (IGoal goal in allGoals) //Assign indexes to the particles in each Goal:
      {
        PS.AssignPIndex(goal, 0.0001); // the second argument is the tolerance distance below which points combine into a single particle
        Print(goal.ToString());
      }
    } else
    {
      //get brep boundary
      Brep brepBoundary = getBoundary(offset, boundary, negative);

      //get columns
      allColumns = getColumns(brepBoundary, numberColumns, fixedColumns);

      //convert brep to mesh
      Rhino.Geometry.Mesh meshBoundary = Rhino.Geometry.Mesh.CreateFromBrep(brepBoundary, MeshingParameters.Default)[0];

      //get goals
      allGoals = getGoals(allColumns, fixedColumns, fixedWalls, meshBoundary, radius);

      foreach (IGoal goal in allGoals) //Assign indexes to the particles in each Goal:
      {
        PS.AssignPIndex(goal, 0.0001); // the second argument is the tolerance distance below which points combine into a single particle
        //Print(goal.ToString());
      }
    }


    int substeps = 5;

    if (run) //PS.GetvSum() > 1e-15 &&
    {
      PS.MomentumStep(allGoals, 0.99, substeps);
      counter += substeps;
      Component.ExpireSolution(true);
    }

    //get the positions of the points
    points = PS.GetPositions();

    //get the number of iterations
    iterations = PS.GetIterations();

    //Print(allGoals.Count.ToString());

  }
  #endregion
  #region Additional
  List<Point3d> allColumns = new List<Point3d>();
  KangarooSolver.PhysicalSystem PS = new KangarooSolver.PhysicalSystem();
  List<IGoal> allGoals = new List<IGoal>();
  int counter = 0;
  //double threshold = 1e-15;

  public Brep getBoundary(double offset, Curve boundary, List<Curve> negative)
  {
    //if offset is positive, then make it negative
    if (offset > 0)
    {
      offset = offset * -1;
    }

    //if offset != 0, then offset the boundary
    if (offset != 0)
    {
      boundary = boundary.Offset(Plane.WorldXY, offset, 0.5, CurveOffsetCornerStyle.Sharp)[0];
    }

    //array size is equal the size of negative list + 1
    Curve[] boundaryCurves = new Curve[negative.Count + 1];
    boundaryCurves[0] = boundary;
    for (int i = 0; i < negative.Count; i++)
    {
      boundaryCurves[i + 1] = negative[i];
    }

    //generate a surface with offseted boundary
    Brep brepBoundary = Brep.CreatePlanarBreps(boundaryCurves, 0.5)[0];

    return brepBoundary;
  }
 
  private List<Point3d> getColumns(Brep brepBoundary, int numberColumns, List<Point3d> fixedColumns)
  {
    //find the center of the boundary
    Point3d center = brepBoundary.GetBoundingBox(false).Center;

    //create a circle at the center of the boundary
    Circle circle = new Circle(center, 0.1);

    //convert the circle to a nurbs curve
    Curve circleCurve = circle.ToNurbsCurve();

    //divide the circle into numberAddColumns
    int numberAddColumns = numberColumns - fixedColumns.Count;

    //create array for extra columns
    Point3d[] addColumns = new Point3d[numberAddColumns];
    double[] test = circleCurve.DivideByCount(numberAddColumns, true, out addColumns);

    //create a list of all columns
    List<Point3d> allColumns = addColumns.ToList();
    for (int i = 0; i < fixedColumns.Count; i++)
    {
      allColumns.Add(fixedColumns[i]);
    }

    return allColumns;
  }
  
  private List<IGoal> getGoals(List<Point3d> allColumns, List<Point3d> fixedColumns, List<Curve> fixedWalls, Rhino.Geometry.Mesh meshBoundary, double radius)
  {
    //onmesh goal
    OnMesh onMesh = new OnMesh(allColumns, meshBoundary, 1000);
    allGoals.Add(onMesh);

    //sphere collide goal
    SphereCollide sphereCollide = new SphereCollide(allColumns, radius, 1);
    allGoals.Add(sphereCollide);

    //anchor goal
    for (int i = 0; i < fixedColumns.Count; i++)
    {
      Anchor anchor = new Anchor(fixedColumns[i], 1000);
      allGoals.Add(anchor);
    }

    //Add fixedWalls constraint here
/*
    List<object> collideList = new List<object>();
    collideList.Add(allColumns);
    collideList.Add(fixedWalls);
    List<double> radiusCollide = new List<double>();
    for (int i = 0; i < collideList.Count; i++)
    {
      radiusCollide.Add(radius);
    }
    Collider collide = new Collider(collideList, radiusCollide, null, null, 1);*/

    return allGoals;
  }
  #endregion
}
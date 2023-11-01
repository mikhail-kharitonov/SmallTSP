using Small_TSP.DataProcessors.Interfaces;
using Small_TSP.DataProcessors;
using Small_TSP.DataModel;
using Small_TSP.Solver;



namespace TravellingSalesmanProblem;

class Program
{
    public static void Main(string[] args)
    {
        string path = "/home/mikhail/RiderProjects/SmallTSP/data/";
        IFileManager fileManager = new FileManager();
        fileManager.Initialize(path);
        string[] files = fileManager.GetFileNames();

        foreach (string file in files)
        {
            string data = fileManager.Read(file);
            ISerializer serializer = new JsonDataManager();
            (List<ArcImprovedRoute> arcsImprovedRoutes, float routeLength, GeoPoint pointStart, GeoPoint pointEnd, List<GeoPoint> solutionVRP) = 
                serializer.Deserialize<(List<ArcImprovedRoute>, float, GeoPoint, GeoPoint, List<GeoPoint>) >(data);
            SolverORTools solver = new SolverORTools();
            int solution = solver.GetSolution(arcsImprovedRoutes, pointStart, pointEnd);
            float diff = (routeLength - solution) / routeLength * 100;

            int recalcVR = solver.GetInitialSolution(arcsImprovedRoutes, pointStart, pointEnd, solutionVRP);
            Console.WriteLine($"VRPResult = {routeLength}, \t ORToolsResult = {solution},  \t recalcVR = {recalcVR},  " +
                              $" \t diff_VRP = {routeLength - recalcVR} \t\t diff = {Math.Round(diff, 2)}%");
        }
        
  

        

        
        //(Dictionary<int, GeoPoint> arcsFrom, Dictionary<int, GeoPoint> arcsTo) = solver.CreateNumbersPoints(arcsImprovedRoutes);
        
      
        //GeoPoint pointStart = new GeoPoint(xStart, yStart);
        //GeoPoint pointEnd = new GeoPoint(xEnd, yEnd);
        
        /*
        //List<int> solution = solver.GetRoutePoints(arcsImprovedRoutes, pointStart, pointEnd);
        Console.WriteLine($"\nMy solution");
        foreach (int item in solution)
        {
            Console.Write($"{item} -> ");
        }
        */
        
    }
}



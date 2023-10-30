using Small_TSP.DataProcessors.Interfaces;
using Small_TSP.DataProcessors;
using Small_TSP.DataModel;
using Small_TSP.Solver;



namespace TravellingSalesmanProblem;

class Program
{
    public static void Main(string[] args)
    {
        string fileName = "C:\\Users\\mikhail\\Axelot\\Small-TSP\\data\\TestCoordinate.json";//"/home/mikhail/RiderProjects/SmallTSP/data/111.json";
        IFileManager fileManager = new FileManager();
        string data = fileManager.Read(fileName);
        ISerializer serializer = new JsonDataManager();

        List<ArcImprovedRoute> arcsImprovedRoutes = serializer.Deserialize<List<ArcImprovedRoute>>(data);

        SolverORTools solver = new SolverORTools();
        (Dictionary<int, GeoPoint> arcsFrom, Dictionary<int, GeoPoint> arcsTo) = solver.CreateNumbersPoints(arcsImprovedRoutes);
        
        double xStart = 55.809762;
        double yStart = 37.392311;
        double xEnd = 55.764592;
        double yEnd = 37.877805;
        GeoPoint pointStart = new GeoPoint(xStart, yStart);
        GeoPoint pointEnd = new GeoPoint(xEnd, yEnd);
        
        
        List<int> solution = solver.GetMaskRoutePoints(arcsImprovedRoutes, pointStart, pointEnd);
        Console.WriteLine($"\nMy solution");
        foreach (int item in solution)
        {
            Console.Write($"{item} -> ");
        }
        

    }
}



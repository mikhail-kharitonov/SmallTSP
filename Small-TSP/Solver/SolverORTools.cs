using Google.OrTools;
using Google.OrTools.ConstraintSolver;
using Small_TSP.DataModel;
using Google.Protobuf.WellKnownTypes;
namespace Small_TSP.Solver;

public class SolverORTools
{
    private static int _amountVehicles = 1;
    private static int _timeLimitForSolutionSeconds = 10;//maxTimeFromImprovment....
    
    private int GetDist(GeoPoint pointFrom, GeoPoint pointTo, List<ArcImprovedRoute> arcs)
    {
        int result = 0;
        IEnumerable<int> distance = arcs
            .Where(a => a.PointFrom.Equals(pointFrom) 
                        && a.PointTo.Equals(pointTo))
            .Select(d => d.Distance);
        
        foreach (int d in distance)
        {
            result =  d;
        }
        return result;
    }

    private int[,] BuldDistance(Dictionary<int, GeoPoint> points, List<ArcImprovedRoute> arcsImprovedRoutes)
    {
        int[,] distance = new int[points.Count, points.Count];
        
        foreach (int row in points.Keys)
        {
            foreach (int column in points.Keys)
            {
                distance[row, column] = GetDist(points[row], points[column], arcsImprovedRoutes);
            }
        }
        return distance;
    }

    private List<long> BuldInitialRoutes(Dictionary<int, GeoPoint> points, List<GeoPoint> initialSolution)
    {
        List<long> numbers = new List<long>(points.Count);
        int number;
        
        foreach (GeoPoint point in initialSolution)
        {
            number = points.FirstOrDefault(p => p.Value.Equals(point)).Key;
            numbers.Add(number);
        }
   
        return numbers;
    }

    public int CalcObjectiveVRP(List<GeoPoint> initialSolution, List<ArcImprovedRoute> arcsImprovedRoutes)
    {
        int distance = 0;
        int numbers = initialSolution.Count;
        for (int i = 0; i < numbers - 1; i++)
        {
            distance = distance + GetDist(initialSolution[i], initialSolution[i + 1], arcsImprovedRoutes);
        }

        return distance;

    }
    public Dictionary<int, GeoPoint> CreateNumbersPoints(List<ArcImprovedRoute> arcsImprovedRoutes)
    {
        Dictionary<int, GeoPoint> points = new Dictionary<int, GeoPoint>();
        
        foreach (ArcImprovedRoute arc in arcsImprovedRoutes)
        {
            if (!points.ContainsValue(arc.PointFrom))
            {
                points.Add(points.Count, arc.PointFrom);
            }
            
            if (!points.ContainsValue(arc.PointTo))
            {
                points.Add(points.Count, arc.PointTo);
            }
            //Собрать int[,], сначала заполнить maxint
        }
        return points;
    }
    private Assignment Solve1(RoutingIndexManager manager, RoutingModel routing, int[,] distanceMatrix, Assignment initialSolution)
    {
        int transitCallbackIndex = routing.RegisterTransitCallback((long fromIndex, long toIndex) =>
        {
            int fromNode = manager.IndexToNode(fromIndex);
            int toNode = manager.IndexToNode(toIndex);
            return distanceMatrix[fromNode, toNode];
        });
        routing.SetArcCostEvaluatorOfAllVehicles(transitCallbackIndex);
       // solution = routing.SolveFromAssignmentWithParameters(initial_solution, search_parameters)
        RoutingSearchParameters searchParameters = operations_research_constraint_solver.DefaultRoutingSearchParameters();
        searchParameters.FirstSolutionStrategy = FirstSolutionStrategy.Types.Value.PathCheapestArc;
        searchParameters.TimeLimit = new Duration { Seconds = _timeLimitForSolutionSeconds };
        return routing.SolveFromAssignmentWithParameters(initialSolution, searchParameters);
        //return routing.SolveWithParameters(initial_solution, searchParameters);
    }
    
    private Assignment Solve(RoutingIndexManager manager, RoutingModel routing, int[,] distanceMatrix)
    {
        int transitCallbackIndex = routing.RegisterTransitCallback((long fromIndex, long toIndex) =>
        {
            int fromNode = manager.IndexToNode(fromIndex);
            int toNode = manager.IndexToNode(toIndex);
            return distanceMatrix[fromNode, toNode];
        });
        routing.SetArcCostEvaluatorOfAllVehicles(transitCallbackIndex);
        // solution = routing.SolveFromAssignmentWithParameters(initial_solution, search_parameters)
        RoutingSearchParameters searchParameters = operations_research_constraint_solver.DefaultRoutingSearchParameters();
        searchParameters.FirstSolutionStrategy = FirstSolutionStrategy.Types.Value.PathCheapestArc;
        searchParameters.TimeLimit = new Duration { Seconds = _timeLimitForSolutionSeconds };
        //return routing.SolveFromAssignmentWithParameters(initialSolution, searchParameters);
        return routing.SolveWithParameters(searchParameters);
    }

    public int GetSolution(List<ArcImprovedRoute> arcsImprovedRoutes, GeoPoint pointStart, GeoPoint pointEnd)
    {
        Dictionary<int, GeoPoint> points = CreateNumbersPoints(arcsImprovedRoutes);

        int[,] distance = BuldDistance(points, arcsImprovedRoutes);
        int startPoint = points.FirstOrDefault(p => p.Value.Equals(pointStart)).Key;
        int endPoint = points.FirstOrDefault(p => p.Value.Equals(pointEnd)).Key;
        int amountNodes = distance.GetLength(0);
        int[] starts = new int [1] { startPoint };
        int[] ends = new int [1] { endPoint };
        
        RoutingIndexManager manager = new RoutingIndexManager(amountNodes, _amountVehicles, starts, ends);
        RoutingModel routing = new RoutingModel(manager);
        Assignment solution = Solve(manager, routing, distance);
        List<int> routePoints = GetRouteNumberPoints(routing, manager, solution, endPoint);
        //Console.WriteLine($"{solution.ObjectiveValue()}");
        return (int)solution.ObjectiveValue(); //routePoints;
    }
    
    private List<int> GetRouteNumberPoints(RoutingModel routing, RoutingIndexManager manager, Assignment solution, int endPoint)
    {
        List<int> routeNumberPoints = new List<int>();
        long index = routing.Start(0);
        while (!routing.IsEnd(index))
        {
            routeNumberPoints.Add(manager.IndexToNode((int)index));
            index = solution.Value(routing.NextVar(index));
        }
        routeNumberPoints.Add(endPoint);
        return routeNumberPoints;
    }
    
    private int[,] BuildOptimalRouteMatrix(RoutingModel routing, Assignment solution, int[,] distanceMatrix)
    {
        long index = routing.Start(0);
        int amount = distanceMatrix.GetLength(0);
        int [,] optimalRouteMatrix = new int[amount, amount];
        long previousIndex;

        while (routing.IsEnd(index) == false)
        {
            previousIndex = index;
            index = solution.Value(routing.NextVar(index));
            if (index <= amount-1)
            {
                optimalRouteMatrix[previousIndex, index] = 1;
            }
            else
            {
                optimalRouteMatrix[previousIndex, routing.Start(0)] = 1;
            }
        }
        return optimalRouteMatrix;
    }


    public int GetInitialSolution(List<ArcImprovedRoute> arcsImprovedRoutes, GeoPoint pointStart, GeoPoint pointEnd, List<GeoPoint> initialRoutesVRP)
    {
        
        Dictionary<int, GeoPoint> points = CreateNumbersPoints(arcsImprovedRoutes);
        //
        List<long> initial = BuldInitialRoutes(points, initialRoutesVRP);
        initial.RemoveAt(0);
        initial.RemoveAt(initial.Count-1);
        long[][] initialRoutes = new long[1][]{initial.ToArray()};
        //
        int[,] distance = BuldDistance(points, arcsImprovedRoutes);
        int startPoint = points.FirstOrDefault(p => p.Value.Equals(pointStart)).Key;
        int endPoint = points.FirstOrDefault(p => p.Value.Equals(pointEnd)).Key;
        int amountNodes = distance.GetLength(0);
        int[] starts = new int [1] { startPoint };
        int[] ends = new int [1] { endPoint };
        
        RoutingIndexManager manager = new RoutingIndexManager(amountNodes, _amountVehicles, starts, ends);
        RoutingModel routing = new RoutingModel(manager);
        Assignment initialSolution = routing.ReadAssignmentFromRoutes(initialRoutes, false);

        Assignment solution = Solve1(manager, routing, distance, initialSolution);
        List<int> routePoints = GetRouteNumberPoints(routing, manager, solution, endPoint);
        //Console.WriteLine($"{solution.ObjectiveValue()}");
        return (int)solution.ObjectiveValue(); //routePoints;
    }
    
    
    
    
}
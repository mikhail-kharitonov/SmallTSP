using Google.OrTools;
using Google.OrTools.ConstraintSolver;
using Small_TSP.DataModel;
namespace Small_TSP.Solver;

public class SolverORTools
{
    private static int _vehicleNumber = 1;
    
    private int GetDist(string arcFrom, string arcTo, List<ArcImprovedRoute> distanceMatrix)
    {
        int result = 0;
        IEnumerable<ArcImprovedRoute> arcs = distanceMatrix.
            Where(d => d.ArcFrom.Equals(arcFrom) && d.ArcTo.Equals(arcTo));

        foreach (ArcImprovedRoute arc in arcs)
        {
            result =  arc.Distance;
        }

        return result;
    }
    

    public int[,] BuildDistanceMatrix(List<ArcImprovedRoute> distanceMatrix)
    {
        HashSet<string> arcsFrom = new HashSet<string>();
        HashSet<string> arcsTo = new HashSet<string>();
        int row = 0;
        int column = 0;

        foreach (ArcImprovedRoute arc in distanceMatrix)
        {
            arcsFrom.Add(arc.ArcFrom);
            arcsTo.Add(arc.ArcTo);
        }
        int[,] distance = new int[arcsFrom.Count, arcsTo.Count];
        
        foreach (string arcFrom in arcsFrom)
        {
            foreach (string arcTo in arcsTo)
            {
                distance[row, column] = GetDist(arcFrom, arcTo, distanceMatrix);
                column++;
            }
            row++;
            column = 0;
        }

        return distance;
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

        RoutingSearchParameters searchParameters = operations_research_constraint_solver.DefaultRoutingSearchParameters();
        searchParameters.FirstSolutionStrategy = FirstSolutionStrategy.Types.Value.PathCheapestArc;
        return routing.SolveWithParameters(searchParameters);
    }

    private int[,] BuildVariables(RoutingModel routing, Assignment solution, int[,] distanceMatrix)
    {
        long index = routing.Start(0);
        int amount = distanceMatrix.GetLength(0);
        int [,] variables = new int[amount, amount];
        long previousIndex;

        while (routing.IsEnd(index) == false)
        {
            previousIndex = index;
            index = solution.Value(routing.NextVar(index));
            if (index <= amount-1)
            {
                variables[previousIndex, index] = 1;
            }
            else
            {
                variables[previousIndex, routing.Start(0)] = 1;
            }
        }
        return variables;
    }

    public (int[,], long) GetSolution(List<ArcImprovedRoute> distanceMatrix, int startPoint)
    {
        int[,] distance = BuildDistanceMatrix(distanceMatrix);
        int count = distance.GetLength(0);
        RoutingIndexManager manager = new RoutingIndexManager(count, _vehicleNumber, startPoint);
        RoutingModel routing = new RoutingModel(manager);
        Assignment solution = Solve(manager, routing, distance);
        int[,] variables = BuildVariables(routing, solution, distance);
        long objective = solution.ObjectiveValue();
        return (variables, objective);
    }

}
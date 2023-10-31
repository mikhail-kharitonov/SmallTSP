using Small_TSP.DataProcessors.Interfaces;

namespace Small_TSP.DataProcessors;

public class FileManager: IFileManager
{
    private string _path;
    private const string _typeFile = "*.json";
    
    public void Initialize(string pathToDir)
    {
        _path = pathToDir;
    }
    
    public string Read(string fileName)
    {
        return File.ReadAllText(fileName);
    }

    public void Write(string fileName, string fileData) 
    {
        File.WriteAllText(fileName, fileData);
    }

    public void Delete(string fileName)
    {
        File.Delete(fileName);
    }
    
    public string[] GetFileNames() 
    {
        return Directory.GetFiles(_path, _typeFile);
    }
}
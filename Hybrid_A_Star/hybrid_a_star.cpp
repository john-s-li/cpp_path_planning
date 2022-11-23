#include <Python.h>

#include "hybrid_a_star.h"

using namespace std;

int main() {
  PyObject *pName, *pModule, *pDict, *pFunc, *pValue;

  string python_source = "reeds_shepp";
  string python_function = "calc_all_paths";

  // Initialize python interpreter 
  Py_Initialize();
  PyRun_SimpleString("import sys");
  PyRun_SimpleString("sys.path.append(\"..\")");

  // Build the name obeject
  pName = PyString_FromString(python_source.c_str());
  if (pName == NULL) {
    cout << "pName: An error occured" << endl;
    PyErr_Print();
    return EXIT_FAILURE;
  }
  // Load module object
  pModule = PyImport_Import(pName);
  if (pModule == NULL) {
    cout << "pModule: An error occured" << endl;
    PyErr_Print();
    return EXIT_FAILURE;
  }

  pDict = PyModule_GetDict(pModule);
  if (pDict == NULL) {
    cout << "pDict: An error occured" << endl;
    PyErr_Print();
    return EXIT_FAILURE;
  }

  pFunc = PyDict_GetItemString(pDict, python_function.c_str());
  if (pFunc == NULL) {
    cout << "pFunc: An error occured" << endl;
    PyErr_Print();
    return EXIT_FAILURE;
  }

  vector<double> args = {0.0, 0.0, 0.0, 0.1, 0.1, 0.1, 0.2, 0.4};

  if (PyCallable_Check(pFunc)) {
    auto pArgs = PyTuple_New(8);
    for (int i = 0; i < args.size(); i++) {
      PyTuple_SetItem(pArgs, i, PyFloat_FromDouble(args[i]));
    }

    pValue = PyObject_CallObject(pFunc, pArgs);

    if (pValue) {
      for (Py_ssize_t i = 0; i < PyList_Size(pValue); ++i) {
        PyObject* next = PyList_GetItem(pValue, i);
        cout << next << endl;
      }
    }
  }
  else {
    cout << "pCallable_Check: An error occured" << endl;
    PyErr_Print();
    return EXIT_FAILURE;
  }



  // Clean Up
  Py_DECREF(pModule);
  Py_DECREF(pName);

  // Finish the python interpreter
  Py_Finalize();

  return 0;
}
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.SceneManagement;

public class Menu : MonoBehaviour
{
    public GameObject popup;
    public GameObject popupfail;
    public GameObject menuPopUp;

    public void enable()
    {
        popup.SetActive(true);
    }
    public void enableFail()
    {
        popupfail.SetActive(true);
    }
    public void StartGame()
    {
        SceneManager.LoadScene(SceneManager.GetActiveScene().buildIndex + 1);
        foreach (GameObject o in GameObject.FindGameObjectsWithTag("Start"))
            Destroy(o);
        Destroy(GameObject.FindGameObjectWithTag("Header"));
    }
    public void popUp()
    {
        popup.SetActive(false);
        if(SceneManager.GetActiveScene().buildIndex !=10)
        SceneManager.LoadScene(SceneManager.GetActiveScene().buildIndex + 1);
        else
        {
            Application.Quit();
        }
    }
    public void popUpFail()
    {
        popupfail.SetActive(false);
        SceneManager.LoadScene(SceneManager.GetActiveScene().buildIndex);
    }
    public void menu()
    {
        menuPopUp.SetActive(true);
    }
    public void menuExit()
    {
        menuPopUp.SetActive(false);
    }
    public void level1()
    {
        SceneManager.LoadScene(2);
        menuExit();
        foreach (GameObject o in GameObject.FindGameObjectsWithTag("Start"))
            Destroy(o);
        Destroy(GameObject.FindGameObjectWithTag("Header"));
    }
    public void level2()
    {
        SceneManager.LoadScene(3);
        menuExit();
        foreach (GameObject o in GameObject.FindGameObjectsWithTag("Start"))
            Destroy(o);
        Destroy(GameObject.FindGameObjectWithTag("Header"));

    }
    public void level3()
    {
        SceneManager.LoadScene(4);
        menuExit();
        foreach (GameObject o in GameObject.FindGameObjectsWithTag("Start"))
            Destroy(o);
        Destroy(GameObject.FindGameObjectWithTag("Header"));
    }
    public void level4()
    {
        SceneManager.LoadScene(5);
        menuExit();
        foreach (GameObject o in GameObject.FindGameObjectsWithTag("Start"))
            Destroy(o);
        Destroy(GameObject.FindGameObjectWithTag("Header"));
    }
    public void level5()
    {
        SceneManager.LoadScene(6);
        menuExit();
        foreach (GameObject o in GameObject.FindGameObjectsWithTag("Start"))
            Destroy(o);
        Destroy(GameObject.FindGameObjectWithTag("Header"));
    }
    public void level6()
    {
        SceneManager.LoadScene(7);
        menuExit();
        foreach (GameObject o in GameObject.FindGameObjectsWithTag("Start"))
            Destroy(o);
        Destroy(GameObject.FindGameObjectWithTag("Header"));
    }
    public void level7()
    {
        SceneManager.LoadScene(8);
        menuExit();
        foreach(GameObject o in GameObject.FindGameObjectsWithTag("Start"))
        Destroy(o);
        Destroy(GameObject.FindGameObjectWithTag("Header"));
    }
    public void level8()
    {
        SceneManager.LoadScene(9);
        menuExit();
        foreach (GameObject o in GameObject.FindGameObjectsWithTag("Start"))
            Destroy(o);
        Destroy(GameObject.FindGameObjectWithTag("Header"));
    }
    public void level9()
    {
        SceneManager.LoadScene(10);
        menuExit();
        foreach (GameObject o in GameObject.FindGameObjectsWithTag("Start"))
            Destroy(o);
        Destroy(GameObject.FindGameObjectWithTag("Header"));
    }
}

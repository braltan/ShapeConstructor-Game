using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.SceneManagement;
using UnityEngine.UI;

public class Manager : MonoBehaviour
{

    // reference to Rigidbody2D component
    Rigidbody2D rb;
    List<GameObject> Children;
    GameObject next;
    int index = 0;
    public float forceAmount;
    public enum ForceType { wind , earthquake,none};
    public enum Direction { right,left,none};
    public Direction direction;
    public ForceType forceType;
    public float timer;
    Text timerText,windText;
     GameObject menu;
    public List<Vector3> scaleList = new List<Vector3>();

    // ball movement not allowed if you touches not the ball at the first time
    bool moveAllowed = true,countDownStarted=false;


    // Use this for initialization
    void Start()
    {
     
        timerText =GameObject.FindGameObjectWithTag("TimeText").GetComponent<Text>();
        windText = GameObject.FindGameObjectWithTag("WindText").GetComponent<Text>();
        Children = new List<GameObject>();
    
        foreach (Transform child in transform)
        {
               Children.Add(child.gameObject);
        }
        next = Children[index];
        rb =next.GetComponent<Rigidbody2D>();

        timerText.text = "";
        windText.text = "";
        if (forceType == ForceType.wind)
        {
            if (direction == Direction.left)
                windText.text = "Wind <- " + forceAmount;
            else if (direction == Direction.right)
                windText.text = "Wind -> " + forceAmount;
        }
        else if (forceType == ForceType.earthquake)
        {
            windText.text = "Earthquake";
        }

    }
    public IEnumerator StartCountdown(float countdownValue)
    {
        timer = countdownValue;
        while (timer > 0)
        {
            timerText.text = timer.ToString();
            yield return new WaitForSeconds(1.0f);
            timer--;
        }
        foreach(GameObject go in Children)
        {
            go.GetComponent<Rigidbody2D>().bodyType = RigidbodyType2D.Kinematic;
        }
        index++;
        GameObject.FindGameObjectWithTag("GameController").GetComponent<Menu>().enable();
        //SceneManager.LoadScene(SceneManager.GetActiveScene().buildIndex+1);
    }
    // Update is called once per frame
    void Update()
    {

        if (index == Children.Count)
        {
            if (!countDownStarted)
            {
                timerText.text = timer.ToString();
              
                StartCoroutine(StartCountdown(timer));
            }
            countDownStarted = true;

            if (forceType == ForceType.wind)
            {
                if (direction == Direction.left)
                {
                    transform.Translate(Input.acceleration.x / 2, 0, 0);               
                    transform.Translate(-Vector2.right.x * forceAmount * Time.deltaTime, 0, 0);
                }
                else if (direction == Direction.right)
                {
                    transform.Translate(Input.acceleration.x / 2, 0, 0);
                    transform.Translate(Vector2.right.x * forceAmount * Time.deltaTime, 0, 0);
                }
            }
            else if ( forceType == ForceType.earthquake)
            {
                int rand = Random.Range(0, 2);
                if(rand == 0) { 
                    transform.Translate(Input.acceleration.x / 2, 0, 0);
                    GameObject.FindGameObjectWithTag("StaticObjects").transform.Translate(-Vector2.right.x * forceAmount * Time.deltaTime, 0, 0);
                }
                else if (rand == 1)
                {
                    transform.Translate(Input.acceleration.x / 2, 0, 0);
                    GameObject.FindGameObjectWithTag("StaticObjects").transform.Translate(Vector2.right.x * forceAmount * Time.deltaTime, 0, 0);
                }
            }

        }

        // Initiating touch event
        // if touch event takes place
        if (index < Children.Count && Input.touchCount > 0)
        {

            // get touch position
            Touch touch = Input.GetTouch(0);


            // obtain touch position
            Vector2 touchPos = Camera.main.ScreenToWorldPoint(touch.position);


            // get touch to take a deal with
            switch (touch.phase)
            {
                // if you touches the screen
                case TouchPhase.Began:
                    next.transform.localScale = new Vector3( scaleList[index].x, scaleList[index].y, scaleList[index].x);
                    rb.bodyType = RigidbodyType2D.Kinematic;
                    break;


                // you move your finger
                case TouchPhase.Moved:
                    // if you touches the ball and movement is allowed then move
                    next.transform.position = new Vector3(touchPos.x, touchPos.y, 0);
                    break;


                // you release your finger
                case TouchPhase.Ended:

                    rb.bodyType = RigidbodyType2D.Dynamic;
                    index++;
                    next = Children[index];
                    rb = next.GetComponent<Rigidbody2D>();
                    break;
            }
        }
    }
}

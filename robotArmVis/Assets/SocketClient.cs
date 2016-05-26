using UnityEngine;
using System.Collections;
using System.Runtime.InteropServices;
using System; 

public class SocketClient : MonoBehaviour {

	[DllImport("ClientConnector")]
	private static extern void TEST();
	[DllImport("ClientConnector")]
	private static extern void clientInit();
	[DllImport("ClientConnector")]
	private static extern void clientDeinit();
	[DllImport("ClientConnector")]
	private static extern int Receive(float[] angle, float[] fingerPos);						//server로 부터 받은 angle을 리턴
	[DllImport("ClientConnector")]
	private static extern void SendResult(int result);
	[DllImport("ClientConnector")]
	private static extern void AngleToPos (float[] angle, float[] pos);		//앵글을 넣어서 forward kinematic를 풀어냄

	float[] Angle = new float[6];									//target angle
	float[] SubAngle = new float[6];								//현재 각도와 타겟 각도의 차이값
	float[] prevAngle = new float[6];								//이전 상태의 각도
	float[] fingerPos = new float[3*3];
	float[] subFinger = new float[3*3];
	float[] prevFinger = new float[3 * 3];

	float[] posAll = new float[3 * 7 + 3 * 3];

	bool isMove, isStarted;
	GameObject[] m_joint = new GameObject[6];
	GameObject[] m_Bone = new GameObject[5];
	GameObject[] m_finger = new GameObject[3];
	float[] JointScale = new float[5];

	public GameObject JointPrefabs;
	public GameObject BonePrefabs;
	public GameObject FingerPrefabs;
	public Camera Cam;
	public GameObject Base;

	int frameLimit = 200;
	public string stringToEdit = "200";
	int frameCount = 0;
	bool collisionCheck;
	private GUIStyle gStyle;

	// Use this for initialization
	void Start () {
		isMove = false;
		isStarted = false;
		collisionCheck = false;

		//GUI Text style Initialize
		gStyle = new GUIStyle();
		gStyle.fontSize = 40;

		//Scale Init
		JointScale [0] = 9.0f;
		JointScale [1] = 9.0f;
		JointScale [2] = 9.0f;
		JointScale [3] = 7.5f;
		JointScale [4] = 12.0f;

		Debug.Log ("Socket Client Start service..");
		TEST ();
		clientInit ();
	}
	
	// Update is called once per frame
	void Update () {
		if (!isMove) {
			if(Receive (Angle, fingerPos) == 1){
				frameCount = 0;
				collisionCheck = false;
				//Initial pos calculate
				if(!isStarted){
					Debug.Log ("Initial set..");
					isStarted = true;

					//Joint & Bone Initialize
					AngleToPos(Angle, posAll);
					for(int i = 0; i < 3*3; i++)
						posAll[7*3+i] = fingerPos[i];
					MakeRobotModel(posAll);

					for(int i = 0; i < 6; i++)
						prevAngle[i] = Angle[i];
					for(int i = 0; i < 3*3; i++)
						prevFinger[i] = fingerPos[i];

					SendResult(0);
				}
				else{
					Debug.Log ("Data received..");
					isMove = true;

					//빠짐값 계산
					for(int i = 0; i < 6; i++)
						SubAngle[i] = Angle[i] - prevAngle[i];
					for(int i = 0; i < 3*3; i++)
						subFinger[i] = fingerPos[i] - prevFinger[i];


					//색초기화
					GameObject.Find("Ground").GetComponent<Renderer>().material.color = Color.white;
					GameObject.Find("Base").GetComponent<Renderer>().material.color = Color.white;
					for(int i = 0; i < 5; i++)
						m_Bone[i].GetComponent<Renderer>().material.color = Color.white;
					for(int i = 0; i < 3; i++)
						m_finger[i].GetComponent<Renderer>().material.color = Color.white;
				}
			}
		} else {

			if(frameCount < frameLimit){
				frameCount++;
				float[] tempAngle = new float[6];
				float[] tempFinger = new float[3*3];
				float[] tempPosAll = new float[3*7 + 3*3];

				for(int i = 0; i < 6; i++)
					tempAngle[i] = prevAngle[i] + (SubAngle[i] / (float)frameLimit * (float)frameCount);
				for(int i = 0; i < 3*3; i++)
					tempFinger[i] = prevFinger[i] + (subFinger[i] / (float)frameLimit * (float)frameCount);

				AngleToPos(tempAngle, tempPosAll);
				for(int i = 0; i < 3*3; i++)
					tempPosAll[7*3+i] = tempFinger[i];
				MoveRobotModel(tempPosAll);
			}
			else{
				//충돌이 났을 때
				if(collisionCheck){
					Debug.Log("Collision Detected!");
					isMove = false;
					SendResult(0);
				
				}else{
					Debug.Log ("send result data..");
					SendResult (1);
				}
				isMove = false;

				for(int i = 0; i < 6; i++)
					prevAngle[i] = Angle[i];
				for(int i = 0; i < 3*3; i++)
					prevFinger[i] = fingerPos[i];
			}

		}
	}

	void OnDestroy(){
		TEST ();
		clientDeinit ();
	}

	void MakeRobotModel(float[] pos){
		//GameObject[] temp = new GameObject[3];

		float length;
		Vector3[] jointPos = new Vector3[7];
		Vector3[] fingerPos = new Vector3[3];
		for (int i = 0; i < 7; i++) {
			jointPos [i] = new Vector3 (pos [3 * i + 0], pos [3 * i + 2] + 18.0f, pos [3 * i + 1]);
		}
		for (int i = 0; i < 3; i++) {
			fingerPos[i] = new Vector3(pos[3*(i+7) + 0], pos[3*(i+7) + 2], pos[3*(i+7) + 1]);
		}

		Vector3[] virtualJoint = new Vector3[6];
		virtualJoint = CreateVirtualJoint(jointPos);

		for (int i = 0; i < 6; i++) {
			m_joint [i] = (GameObject)Instantiate (JointPrefabs, virtualJoint [i], new Quaternion (0.0f, 0.0f, 0.0f, 0.0f));
			m_joint [i].transform.parent = this.transform;
		}
		
		//Bone Initialize
		for (int i = 0; i < 5; i++) {
			m_Bone[i] = (GameObject)Instantiate(BonePrefabs, (virtualJoint[i+1] - virtualJoint[i])/2 + virtualJoint[i], new Quaternion(0.0f, 0.0f, 0.0f, 0.0f));
			length = Mathf.Abs((virtualJoint[i+1] - virtualJoint[i]).magnitude) / 2.0f/* + JointScale[i]*/;
			m_Bone[i].transform.localScale = new Vector3(JointScale[i], length, JointScale[i]);
			m_Bone[i].transform.up = (virtualJoint[i+1] - virtualJoint[i]).normalized;
			m_Bone[i].transform.parent = this.transform;
			//m_Bone[i].GetComponent<BoneScript>().setID(i+1);
		}

		//SetID - BODY
		m_Bone[0].GetComponent<BoneScript>().setID(1);
		m_Bone[1].GetComponent<BoneScript>().setID(1);
		m_Bone[2].GetComponent<BoneScript>().setID(2);
		m_Bone[3].GetComponent<BoneScript>().setID(2);
		m_Bone[4].GetComponent<BoneScript>().setID(3);

		//Finger Initialize
		Vector3 center = new Vector3 ((fingerPos [0].x + fingerPos [1].x) / 2, (fingerPos [0].y + fingerPos [1].y) / 2, (fingerPos [0].z + fingerPos [1].z) / 2);
		center.Normalize();
		Vector3 orth, proj;
		float projLength;
		orth = Vector3.Cross (center, fingerPos [2]).normalized;
			
		//Left & Right
		for (int i = 0; i < 2; i++) {
			projLength = Vector3.Dot (fingerPos [i], orth);
			proj = orth * projLength;

			length = (fingerPos[i] - proj).magnitude / 2.0f;
			m_finger[i] = (GameObject)Instantiate(FingerPrefabs, (fingerPos[i] - proj) / 2.0f +virtualJoint[5] + proj, new Quaternion(0.0f, 0.0f, 0.0f, 0.0f));
			m_finger[i].transform.localScale = new Vector3(2.0f, length, 2.0f);
			m_finger[i].transform.up = (fingerPos[i] - proj).normalized;
			m_finger[i].transform.parent = this.transform;
		}
		
		//Thumb
		m_finger[2] = (GameObject)Instantiate(FingerPrefabs, virtualJoint[5]+(fingerPos[2]/2.0f), new Quaternion(0.0f, 0.0f, 0.0f, 0.0f));
		length = Mathf.Abs(fingerPos[2].magnitude) / 2.0f;
		m_finger[2].transform.localScale = new Vector3(2.0f, length, 2.0f);
		m_finger[2].transform.up = fingerPos[2].normalized;
		m_finger[2].transform.parent = this.transform;
		
		//SetID - Finger
		m_finger[0].GetComponent<BoneScript>().setID(4);
		m_finger[1].GetComponent<BoneScript>().setID(4);
		m_finger[2].GetComponent<BoneScript>().setID(4);
	}

	void MoveRobotModel(float[] pos){
		float length;
		Vector3[] jointPos = new Vector3[7];
		Vector3[] fingerPos = new Vector3[3];
		for (int i = 0; i < 7; i++) {
			jointPos [i] = new Vector3 (pos [3 * i + 0], pos [3 * i + 2] + 18.0f, pos [3 * i + 1]);
			//m_joint [i] = (GameObject)Instantiate (JointPrefabs, jointPos [i], new Quaternion (0.0f, 0.0f, 0.0f, 0.0f));
		}
		for (int i = 0; i < 3; i++) {
			fingerPos[i] = new Vector3(pos[3*(i+7) + 0], pos[3*(i+7) + 2], pos[3*(i+7) + 1]);
		}
		
		Vector3[] virtualJoint = new Vector3[6];
		virtualJoint = CreateVirtualJoint(jointPos);
		
		for (int i = 0; i < 6; i++)
			m_joint[i].transform.position = virtualJoint[i];
		
		//Bone Initialize
		for (int i = 0; i < 5; i++) {
			m_Bone[i].transform.position = (virtualJoint[i+1] - virtualJoint[i])/2 + virtualJoint[i];
			//float length = Mathf.Abs((virtualJoint[i+1] - virtualJoint[i]).magnitude) / 2.0f/* + JointScale[i]*/;
			//m_Bone[i].transform.localScale = new Vector3(JointScale[i], length, JointScale[i]);
			m_Bone[i].transform.up = (virtualJoint[i+1] - virtualJoint[i]).normalized;
		}

		//Finger Initialize
		Vector3 center = new Vector3 ((fingerPos [0].x + fingerPos [1].x) / 2, (fingerPos [0].y + fingerPos [1].y) / 2, (fingerPos [0].z + fingerPos [1].z) / 2);
		center.Normalize();
		Vector3 orth, proj;
		float projLength;
		orth = Vector3.Cross (center, fingerPos [2]).normalized;
		
		//Left & Right
		for (int i = 0; i < 2; i++) {
			projLength = Vector3.Dot (fingerPos [i], orth);
			proj = orth * projLength;
			
			length = (fingerPos[i] - proj).magnitude / 2.0f;
			m_finger[i].transform.position = (fingerPos[i] - proj) / 2.0f +virtualJoint[5] + proj;
			m_finger[i].transform.localScale = new Vector3(2.0f, length, 2.0f);
			m_finger[i].transform.up = (fingerPos[i] - proj).normalized;
		}
		
		//Thumb
		m_finger [2].transform.position = virtualJoint [5] + (fingerPos [2] / 2.0f);
		length = Mathf.Abs(fingerPos[2].magnitude) / 2.0f;
		m_finger[2].transform.localScale = new Vector3(2.0f, length, 2.0f);
		m_finger[2].transform.up = fingerPos[2].normalized;
	}

	Vector3[] CreateVirtualJoint(Vector3[] jointPos){
		Vector3[] retArray = new Vector3[6];
		retArray [0] = jointPos [0];

		float length = Mathf.Abs((jointPos [4] - jointPos [3]).magnitude);
		float tempLength = Mathf.Abs((jointPos [4] - jointPos [2]).magnitude);
		retArray [1] = ((jointPos [4] - jointPos [2]).normalized * (tempLength - length)) + retArray [0];
		retArray [2] = jointPos [3];
		retArray [4] = jointPos [5];
		retArray [5] = jointPos [6];
		retArray[3] = retArray[1] + 2*(jointPos[4] - retArray[1]);

		return retArray;
	}

	public void CollisionDetected(){
		Debug.Log ("Collision Detected!");
		collisionCheck = true;
	}

	void OnGUI(){
		Vector3 t = Cam.WorldToScreenPoint (Base.transform.position);

		if (collisionCheck) {
			GUI.Label(new Rect(t.x, UnityEngine.Screen.height - t.y, 400, 400), "Collision detected", gStyle);
		} else {
			GUI.Label(new Rect(t.x, UnityEngine.Screen.height - t.y, 400, 400), "Safe", gStyle);
		}

		stringToEdit = GUI.TextField(new Rect(10f, 10f, 200f, 50f), stringToEdit, 25);
		if (GUI.Button(new Rect(220f, 10f, 50f, 50f), "Adapt"))
		{
			frameLimit = Int32.Parse(stringToEdit);
		}
	}
}

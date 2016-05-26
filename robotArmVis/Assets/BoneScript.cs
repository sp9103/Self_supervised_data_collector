using UnityEngine;
using System.Collections;

public class BoneScript : MonoBehaviour {
	public int BoneID = 0;

	private BoneScript script;
	// Use this for initialization
	void Start () {
	}
	
	// Update is called once per frame
	void Update () {
	
	}

	void OnTriggerEnter(Collider other){
		script = other.gameObject.GetComponent<BoneScript> ();
		int otherID = script.BoneID;

		if (otherID != BoneID) {
			int beforeID = BoneID - 1;
			int afterID = BoneID + 1;

			if(otherID != beforeID && otherID != afterID){
				//부모 오브젝트의 충돌 여부를 알림
				Debug.Log ("["+BoneID+"]"+" "+otherID);
				this.GetComponentInParent<SocketClient>().CollisionDetected();
				this.GetComponent<Renderer>().material.color = Color.red;
			}
		}

	}

	public void setID(int id){
		BoneID = id;
	}
}

using UnityEngine;
using System.Collections;
using System.Collections.Generic;

public class Pathfinding : MonoBehaviour {

	public GameObject pathprefab;
	private float[,] perlingrid;
	private float slopelim = 10.0f;
	private float waterlevel;

	// Use this for initialization
	void Start () {
	}
	
	// Update is called once per frame
	void Update () {
	
	}

	public IEnumerator pathfindwrapper(Vector3 v3start, Vector3 v3end, Vector3 targetdim, System.Action<List<Vector3>> retpathact){
		//Debug.Log ("pathfind start");
		Camera.main.GetComponent<CoreScript> ().addunittoPF ();
		List<Vector3> tempcurpath = new List<Vector3> ();
		yield return StartCoroutine(pathfind(v3start,v3end,targetdim, (x) => tempcurpath=x));
		retpathact (tempcurpath);
		Camera.main.GetComponent<CoreScript> ().subunittoPF ();
		//Debug.Log ("pathfind end");
	}


	public IEnumerator pathfind(Vector3 v3start, Vector3 v3end, Vector3 targetdim, System.Action<List<Vector3>> retpathact){
		perlingrid = Camera.main.GetComponent<TerrainGen> ().getperlingrid();
		waterlevel = Camera.main.GetComponent<TerrainGen> ().getwaterlevel ();

		//moved declarations to private
		HashSet<Vector3> openset = new HashSet<Vector3> ();
		HashSet<Vector3> closedset = new HashSet<Vector3> ();
		Dictionary<Vector3,Vector3> navigated=new Dictionary<Vector3,Vector3>();
		Dictionary<Vector3,float> gscore=new Dictionary<Vector3,float>();
		Dictionary<Vector3,float> fscore=new Dictionary<Vector3,float>();
		List<Vector3> curpath = new List<Vector3> ();

		openset.Add (v3start);

		gscore [v3start] = 0.0f;
		fscore [v3start] = gscore [v3start] + hscore (v3start, v3end);

		while (openset.Count>0) {
			Vector3 current=minv3(openset,fscore);

			//GameObject pathobj = Instantiate (pathprefab, current, Quaternion.identity) as GameObject;
			//pathobj.GetComponent<Renderer> ().material.color = Color.white;
			//pathobj.transform.localScale=new Vector3(1.0f,5.0f,1.0f);

			if (eqpos(current,v3end, targetdim)){
				//Debug.Log (v3end);
				reconstructnav(navigated,current,ref curpath);
				retpathact (curpath);
				//showpath(curpath);
				yield break;
			}
			openset.Remove (current);
			closedset.Add (current);

			List<Vector3> jumppts=retjumppts(current,v3start,v3end,targetdim, navigated);

			foreach(Vector3 jumppt in jumppts){
				if (closedset.Contains (jumppt)){;
					continue;
				}
				float tempg=gscore[current]+hscore (current,jumppt);
				if ((!(openset.Contains (jumppt))) || (gscore.ContainsKey(jumppt) && (tempg<gscore[jumppt]))){
					navigated[jumppt]=current;
					//show every branch - showpath (new List<Vector3>(){current,jumppt});
					gscore[jumppt]=tempg;
					fscore[jumppt]=gscore[jumppt]+hscore (jumppt,v3end);
					if (!(openset.Contains(jumppt))){
						openset.Add(jumppt);
					}
				}
				yield return null;
			}
		}
		curpath.Add (v3start);
		retpathact (curpath);
		Debug.Log("no path found");
	}

	private List<Vector3> retjumppts(Vector3 v3in,Vector3 v3start, Vector3 v3end,Vector3 targetdim, Dictionary<Vector3,Vector3> navigated){
		List<Vector3> successors = new List<Vector3> ();
		List<Vector3> neighbors = getneighbors(v3in,v3start,navigated);
		foreach (Vector3 neighbor in neighbors){
			Vector3 tempnode = jump(v3in,nodedir(v3in,neighbor),v3end, targetdim);
			if (tempnode!=new Vector3(0.0f,0.0f,0.0f)){
				successors.Add(tempnode);
			}
		}
		return successors;
	}


	private Vector3 jump(Vector3 curpos,Vector2 dir, Vector3 v3end, Vector3 targetdim){
		Vector3 temppos = new Vector3 (curpos.x + dir.x, 0, curpos.z + dir.y);
		if (!(validneighbor (curpos, temppos))) {
			return new Vector3 (0.0f, 0.0f, 0.0f);
		}
		temppos=new Vector3(temppos.x,perlingrid[(int)temppos.x,(int)temppos.z],temppos.z);
		if (eqpos(temppos,v3end, targetdim)) {
			return temppos;
		} 
		else if (straightforced (curpos, temppos, dir)) {
			return temppos;
		} 
		else if ((dir.x != 0.0f) && (dir.y != 0.0f)) {
			if (jump (temppos,new Vector2(dir.x,0.0f),v3end, targetdim)!=new Vector3(0.0f,0.0f,0.0f)){
				return temppos;
			}
			if (jump (temppos,new Vector2(0.0f,dir.y),v3end, targetdim)!=new Vector3(0.0f,0.0f,0.0f)){
				return temppos;
			}
		}
		return jump (temppos,dir,v3end, targetdim);
	}

	private bool straightforced(Vector3 curpos, Vector3 newpos, Vector2 dir){
		// north/south
		if (dir.x == 0) {
			//left is invalid, and left + xdir is valid
			if ((!(validneighbor(newpos,new Vector3(newpos.x-1.0f,0.0f,newpos.z)))) && (validneighbor(newpos,new Vector3(newpos.x-1.0f,0.0f,newpos.z+dir.y)))){
				return true;
			}
			//right is invalid and right+ydir is valid
			if ((!(validneighbor(newpos,new Vector3(newpos.x+1.0f,0.0f,newpos.z)))) && (validneighbor(newpos,new Vector3(newpos.x+1.0f,0.0f,newpos.z+dir.y)))){
				return true;
			}
		}
		// west/right
		else if (dir.y == 0) {
			//down is invalid, and down+ydir is valid
			if ((!(validneighbor(newpos,new Vector3(newpos.x,0.0f,newpos.z-1.0f)))) && (validneighbor(newpos,new Vector3(newpos.x+dir.x,0.0f,newpos.z+dir.y-1.0f)))){
				return true;
			}
			//up is valid, and up+ydir is valid
			if ((!(validneighbor(newpos,new Vector3(newpos.x,0.0f,newpos.z+1.0f)))) && (validneighbor(newpos,new Vector3(newpos.x+dir.x,0.0f,newpos.z+dir.y+1.0f)))){
				return true;
			}
		} 
		return false;
	}

	//direction of node in Vector2 form (should be (0~1,0~1))
	private Vector2 nodedir(Vector3 curpos, Vector3 nxtpos){
		return new Vector2 (Mathf.Clamp (nxtpos.x - curpos.x, -1.0f, 1.0f), Mathf.Clamp (nxtpos.z - curpos.z, -1.0f, 1.0f));
	}

	private List<Vector3> getneighbors(Vector3 v3in, Vector3 v3start, Dictionary<Vector3,Vector3> navigated){
		List<Vector3> retneighbors = new List<Vector3> ();
		Vector2 travdir;
		if (v3in == v3start) {
			travdir = new Vector2 (0.0f, 0.0f);
		} else {
			travdir=nodedir (navigated[v3in],v3in);
		}

		//starting node
		if (travdir.x==0.0f && travdir.y==0.0f){
			//add all 8 nodes if valid
			if (validneighbor (v3in,new Vector3(v3in.x+1.0f,0,v3in.z+1.0f))){
				retneighbors.Add(new Vector3(v3in.x+1.0f,perlingrid[(int)(v3in.x+1.0f),(int)(v3in.z+1.0f)],v3in.z+1.0f));
			}
			if (validneighbor (v3in,new Vector3(v3in.x+1.0f,0,v3in.z))){
				retneighbors.Add(new Vector3(v3in.x+1.0f,perlingrid[(int)(v3in.x+1.0f),(int)(v3in.z)],v3in.z));
			}
			if (validneighbor (v3in,new Vector3(v3in.x+1.0f,0,v3in.z-1.0f))){
				retneighbors.Add(new Vector3(v3in.x+1.0f,perlingrid[(int)(v3in.x+1.0f),(int)(v3in.z-1.0f)],v3in.z-1.0f));
			}
			if (validneighbor (v3in,new Vector3(v3in.x,0,v3in.z+1.0f))){
				retneighbors.Add(new Vector3(v3in.x,perlingrid[(int)(v3in.x),(int)(v3in.z+1.0f)],v3in.z+1.0f));
			}
			if (validneighbor (v3in,new Vector3(v3in.x,0,v3in.z-1.0f))){
				retneighbors.Add(new Vector3(v3in.x,perlingrid[(int)(v3in.x),(int)(v3in.z-1.0f)],v3in.z-1.0f));
			}
			if (validneighbor (v3in,new Vector3(v3in.x-1.0f,0,v3in.z+1.0f))){
				retneighbors.Add(new Vector3(v3in.x-1.0f,perlingrid[(int)(v3in.x-1.0f),(int)(v3in.z+1.0f)],v3in.z+1.0f));
			}
			if (validneighbor (v3in,new Vector3(v3in.x-1.0f,0,v3in.z))){
				retneighbors.Add(new Vector3(v3in.x-1.0f,perlingrid[(int)(v3in.x-1.0f),(int)(v3in.z)],v3in.z));
			}
			if (validneighbor (v3in,new Vector3(v3in.x-1.0f,0,v3in.z-1.0f))){
				retneighbors.Add(new Vector3(v3in.x-1.0f,perlingrid[(int)(v3in.x-1.0f),(int)(v3in.z-1.0f)],v3in.z-1.0f));
			}
		}
		//vertical
		else if (travdir.x==0.0f && travdir.y!=0.0f){
			//add the forward pos
			if (validneighbor(v3in,new Vector3(v3in.x,0.0f,v3in.z+travdir.y))){
				retneighbors.Add (new Vector3(v3in.x,perlingrid[(int)(v3in.x),(int)(v3in.z+travdir.y)],v3in.z+travdir.y));
			}
			//check for sideblocks and add the corner block
			//left sideblock
			if (!validneighbor(v3in,new Vector3(v3in.x-1.0f,0.0f,v3in.z)) && validneighbor(v3in,new Vector3(v3in.x-1.0f,0.0f,v3in.z+travdir.y))){
				retneighbors.Add (new Vector3(v3in.x-1.0f,perlingrid[(int)(v3in.x-1.0f),(int)(v3in.z+travdir.y)],v3in.z+travdir.y));
			}
			//right sideblock
			if (!validneighbor(v3in,new Vector3(v3in.x+1.0f,0.0f,v3in.z)) && validneighbor(v3in,new Vector3(v3in.x+1.0f,0.0f,v3in.z+travdir.y))){
				retneighbors.Add (new Vector3(v3in.x+1.0f,perlingrid[(int)(v3in.x+1.0f),(int)(v3in.z+travdir.y)],v3in.z+travdir.y));
			}
		}
		//horizontal
		else if (travdir.x!=0.0f && travdir.y==0.0f){
			//add the forward pos
			if (validneighbor(v3in,new Vector3(v3in.x+travdir.x,0.0f,v3in.z))){
				retneighbors.Add (new Vector3(v3in.x+travdir.x,perlingrid[(int)(v3in.x+travdir.x),(int)(v3in.z)],v3in.z));
			}
			//check for sideblocks and add the corner block
			//up sideblock
			if (!validneighbor(v3in,new Vector3(v3in.x,0.0f,v3in.z+1.0f)) && validneighbor(v3in,new Vector3(v3in.x+travdir.x,0.0f,v3in.z+1.0f))){
				retneighbors.Add (new Vector3(v3in.x+travdir.x,perlingrid[(int)(v3in.x+travdir.x),(int)(v3in.z+1.0f)],v3in.z+1.0f));
			}
			//down sideblock
			if (!validneighbor(v3in,new Vector3(v3in.x,0.0f,v3in.z-1.0f)) && validneighbor(v3in,new Vector3(v3in.x+travdir.x,0.0f,v3in.z-1.0f))){
				retneighbors.Add (new Vector3(v3in.x+travdir.x,perlingrid[(int)(v3in.x+travdir.x),(int)(v3in.z-1.0f)],v3in.z-1.0f));
			}
		}
		//diagonal
		else{
			if(validneighbor(v3in,new Vector3(v3in.x+travdir.x,0,v3in.z))){
				retneighbors.Add (new Vector3(v3in.x+travdir.x,perlingrid[(int)(v3in.x+travdir.x),(int)(v3in.z)],v3in.z));
			}
			if(validneighbor(v3in,new Vector3(v3in.x,0,v3in.z+travdir.y))){
				retneighbors.Add (new Vector3(v3in.x,perlingrid[(int)(v3in.x),(int)(v3in.z+travdir.y)],v3in.z+travdir.y));
			}
			if(validneighbor(v3in,new Vector3(v3in.x+travdir.x,0,v3in.z+travdir.y))){
				retneighbors.Add (new Vector3(v3in.x+travdir.x,perlingrid[(int)(v3in.x+travdir.x),(int)(v3in.z+travdir.y)],v3in.z+travdir.y));
			}
			//check the blocked nodes
			//horizontal
			if (!validneighbor(v3in,new Vector3(v3in.x-travdir.x,0.0f,v3in.z)) && validneighbor(v3in,new Vector3(v3in.x-travdir.x,0.0f,v3in.z+travdir.y))){
				retneighbors.Add (new Vector3(v3in.x-travdir.x,perlingrid[(int)(v3in.x-travdir.x),(int)(v3in.z+travdir.y)],v3in.z+travdir.y));
			}
			//vertical
			if (!validneighbor(v3in,new Vector3(v3in.x,0.0f,v3in.z-travdir.y)) && validneighbor(v3in,new Vector3(v3in.x+travdir.x,0.0f,v3in.z-travdir.y))){
				retneighbors.Add (new Vector3(v3in.x+travdir.x,perlingrid[(int)(v3in.x+travdir.x),(int)(v3in.z-travdir.y)],v3in.z-travdir.y-travdir.y));
			}
		}

		return retneighbors;
	}

	//return if tempnode is a valid neighbor of curnode
	private bool validneighbor(Vector3 curnode, Vector3 tempnode){
		int mapdim = Camera.main.GetComponent<TerrainGen> ().getdim ();
		if (tempnode.x < 0 || tempnode.z < 0 || tempnode.x > mapdim || tempnode.z > mapdim) {
			return false;
		} else if (Mathf.Abs (perlingrid [(int)curnode.x, (int)curnode.z] - perlingrid [(int)tempnode.x, (int)tempnode.z]) > slopelim) {
			Debug.Log ("slopechecked");
			return false;
		} 
		else if (perlingrid [(int)tempnode.x, (int)tempnode.z] < waterlevel) {
			return false;
		} 
		else if (!(accessible ((int)tempnode.x,(int)tempnode.z))) {
			return false;
		}
		else{
			return true;
		}
	}

	private float hscore(Vector3 v3_1, Vector3 v3_2){
		return Vector2.Distance (new Vector2 (v3_1.x, v3_1.z), new Vector2 (v3_2.x, v3_2.z));
	}

	private Vector3 minv3(HashSet<Vector3> inopen, Dictionary<Vector3,float> infscore){
		float minval = Mathf.Infinity;
		float[] tempf = new float[inopen.Count];
		Vector3 retminv3 = new Vector3 ();
		int i = 0;
		foreach(Vector3 entry in inopen){
			tempf[i]=infscore[entry];
			if (tempf[i]<minval){
				minval=tempf[i];
				retminv3=entry;
			}
			i++;
		}
		return retminv3;
	}
	private bool eqpos(Vector3 v3in,Vector3 target, Vector3 targetdim){
		if (((v3in.x == target.x) && (v3in.z == target.z)) && targetdim == new Vector3(0.0f,0.0f,0.0f)) {
			return true;
		} 
		else if((v3in.x>=((int)target.x-(((int)targetdim.x+1)/2))) && (v3in.x<=((int)target.x+(((int)targetdim.x+1)/2))) && (v3in.z>=((int)target.z-(((int)targetdim.z+1)/2))) && (v3in.z<=((int)target.z+(((int)targetdim.z+1)/2)))){
			return true;
		}
		return false;
	}

	private void reconstructnav(Dictionary<Vector3,Vector3> navigated, Vector3 curv3, ref List<Vector3> curpath){
		curpath.Add (curv3);
		while (navigated.ContainsKey (curv3)) {
			curv3 = navigated [curv3];
			curpath.Add (curv3);
		}
		//smoothpath (ref curpath);
	}

	private void smoothpath(ref List<Vector3> curpath){
		for (int i=0; i<curpath.Count; i++) {
			Debug.Log ("i: "+i);
			for (int j=curpath.Count-1;j>(i+1);j--){
				Debug.Log ("j: "+j);
				if (straightpathexists(curpath[i],curpath[j])){
					for (int k=i+1;k<j;k++){
						Debug.Log ("k: "+k);
						curpath.Remove (curpath[k]);
					}
				}
			}
		}
	}

	private bool straightpathexists(Vector3 startnode, Vector3 endnode){
		Vector2 travdir = nodedir (startnode, endnode);
		Vector3 curnode = startnode;
		Vector3 prevnode = startnode;
		while (curnode!=endnode) {
			prevnode=curnode;
			curnode=new Vector3(curnode.x+travdir.x,perlingrid[(int)(curnode.x+travdir.x),(int)(curnode.z+travdir.y)],curnode.z+travdir.y);
			if (!validneighbor(prevnode,curnode)){
				return false;
			}
		}
		return true;
	}

	private void showpath(List<Vector3> pathin){
		for (int i=0; i<pathin.Count-1; i++) {
			Vector3 curnode=pathin[i];
			Vector3 nextnode=pathin[i+1];
			Vector2 dir=nodedir (curnode,nextnode);
			while (curnode != nextnode){
				curnode=new Vector3(curnode.x+dir.x,perlingrid[(int)(curnode.x+dir.x),(int)(curnode.z+dir.y)],curnode.z+dir.y);
				GameObject pathobject = Instantiate (pathprefab, curnode, Quaternion.identity) as GameObject;
				pathobject.GetComponent<Renderer> ().material.color = Color.green;
			}
		}
	}

	public float getslopelim(){
		return slopelim;
	}

	private bool accessible(int xin, int zin){
		int perlindim = Camera.main.GetComponent<TerrainGen> ().getdim();
		int[,] accgrid = Camera.main.GetComponent<CoreScript> ().getaccgrid ();
		if (xin<perlindim && zin<perlindim && accgrid[xin,zin]!=1){
			return false;
		}
		else if ((xin-1)>=0 && zin<perlindim && accgrid[xin-1,zin]!=1){
			return false;
		}
		else if (xin < perlindim && (zin-1)>=0 && accgrid[xin,zin-1]!=1){
			return false;
		}
		else if ((xin-1)>=0 && (zin-1)>=0 && accgrid[xin-1,zin-1]!=1){
			return false;
		}
		return true;
	}

}

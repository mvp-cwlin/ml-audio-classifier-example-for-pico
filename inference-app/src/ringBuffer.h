/***** ringBuffer.hpp *****/
class ringBuffer{
	private:
		float* data;
		int size;
		int curIndex = 0;
	
	public:
		ringBuffer(int s){
			data = new float[s];
			size = s;
			for(int i = 0; i < size; i++)
				data[i] = 0;
		}
		
		void push(float d){
			data[curIndex++] = d;
			if(curIndex == size)
				curIndex = 0;
		}
		
		float front(){
			int index = curIndex;
			if(index == size)
				index = 0;
			return data[index];
		}
		
		void reset(){
			for(int i = 0; i < size; i++){
				data[i] = 0;
			}
			
			curIndex = 0;
		}
};
/**************************************************
*
*   FSM
*
**************************************************/

FSM(HeirarchyExample)
{
	FSM_STATES
	{
		X,
		Y		
	}
	
	FSM_START(X);
	FSM_BGN
	{
		FSM_STATE(X)
		{
			FSM_CALL_FSM(OffSpring)
			FSM_TRANSITIONS
			{
				FSM_ON_EVENT(/B, FSM_NEXT(Y));
			}
		} 
							
		FSM_STATE(Y)
		{
			FSM_CALL_FSM(OffSpring2)
		} 
				}
	FSM_END
}




FSM(OffSpring)
{
	FSM_STATES
	{
		OffSpring/A,
		OffSpring/B		
	}
	
	FSM_START(OffSpring/B);
	FSM_BGN
	{
		FSM_STATE(OffSpring/A)
		{
			FSM_CALL_TASK(TASK[Loop])
			FSM_TRANSITIONS
			{
				FSM_ON_EVENT(/Close, FSM_NEXT(OffSpring/B));
				FSM_ON_EVENT(/Loop, FSM_NEXT(OffSpring/A));
			}
		}
				
		FSM_STATE(OffSpring/B)
		{
			FSM_CALL_TASK(TASK[Close])
		}
	}
	FSM_END
}

//**********************************************************
			
FSM(OffSpring2)
{
	FSM_STATES
	{
		A,
		B		
	}
	
	FSM_START(A);
	FSM_BGN
	{
		FSM_STATE(A)
		{
			FSM_CALL_TASK(TASK[Loop])
			FSM_TRANSITIONS
			{
				FSM_ON_EVENT(/Close, FSM_NEXT(B));
				FSM_ON_EVENT(/Loop, FSM_NEXT(A));
			}
		}
				
		FSM_STATE(B)
		{
			FSM_CALL_TASK(TASK[Close])
		}
	}
	FSM_END
}

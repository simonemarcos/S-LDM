#ifndef OPTIONALDATAITEM_H
#define OPTIONALDATAITEM_H	

// Class to store optional data
// If the data is not available, m_available is 'false' and no actual data is stored (getData() does not return any meaningful data)
// If the data is available (isAvailable() returns 'true'), then the actual data can be retrieved with getData()
namespace ldmmap{
	template <class T> class OptionalDataItem {
		private:
		bool m_available;
		T m_dataitem;

		public:
		OptionalDataItem(T data): m_dataitem(data) {m_available=true;}
		OptionalDataItem(bool availability) {m_available=availability;}
		OptionalDataItem() {m_available=false;}
		T getData() {return m_dataitem;}
		bool isAvailable() {return m_available;}
		T setData(T data) {m_dataitem=data; m_available=true;}
	};
}

#endif // OPTIONALDATAITEM_H